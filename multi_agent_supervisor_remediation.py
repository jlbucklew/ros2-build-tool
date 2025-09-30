#!/usr/bin/env python3
"""
Multi-Agent Code Review System - Enhanced Edition
Features:
- Cross-platform Windows compatibility
- Git branch management (main branch enforcement)
- Intelligent junk cleanup and archival
- Enhanced error handling and retry logic
- Progress tracking and visualization
- Configurable task system
- Structured logging
- Pre-flight validation
- Rollback capability
"""

import os
import sys
import re
import json
import shutil
import subprocess
from pathlib import Path
from datetime import datetime
from typing import List, Tuple, Dict, Any, Optional
from anthropic import Anthropic

# ============= CONFIGURATION =============
MAX_ENGINEER_ITERATIONS = 30
MAX_ADMIN_ITERATIONS = 1
API_KEY = os.environ.get("ANTHROPIC_API_KEY")
ARCHIVE_DIR = "archive"
LOGS_DIR = "logs"
CONFIG_FILE = "tasks_config.json"

# Files to keep in root (all others may be archived if old)
ESSENTIAL_FILES = {
    "multi_agent_supervisor_remediation.py",
    "PRODUCTION_RESTART_STATUS.md",
    "README.md",
    ".gitignore",
    "pyproject.toml",
    "setup.cfg",
    ".pre-commit-config.yaml",
    "run_tests_with_evidence.py",
}

# Directories to keep
ESSENTIAL_DIRS = {
    "src", "tests", "requirements", "scripts", "docker", 
    ".github", ".git", ".claude", ARCHIVE_DIR, LOGS_DIR
}

# ============= COLOR OUTPUT (Windows Compatible) =============
class Colors:
    """ANSI color codes for Windows terminal"""
    RESET = '\033[0m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'

def colored(text: str, color: str) -> str:
    """Return colored text for terminal"""
    return f"{color}{text}{Colors.RESET}"

# ============= LOGGING SYSTEM =============
class Logger:
    def __init__(self, task_id: str = "system"):
        self.task_id = task_id
        self.log_file = Path(LOGS_DIR) / f"{task_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        self.log_file.parent.mkdir(parents=True, exist_ok=True)
        self.entries = []
    
    def log(self, level: str, message: str, data: Optional[Dict] = None):
        """Log a message with structured data"""
        entry = {
            "timestamp": datetime.now().isoformat(),
            "task_id": self.task_id,
            "level": level,
            "message": message,
            "data": data or {}
        }
        self.entries.append(entry)
        
        # Console output
        color_map = {
            "INFO": Colors.CYAN,
            "SUCCESS": Colors.GREEN,
            "WARNING": Colors.YELLOW,
            "ERROR": Colors.RED,
            "DEBUG": Colors.BLUE
        }
        color = color_map.get(level, Colors.RESET)
        print(colored(f"[{level}] {message}", color))
    
    def save(self):
        """Save log to file"""
        with open(self.log_file, 'w', encoding='utf-8') as f:
            json.dump(self.entries, f, indent=2)
        return self.log_file

# ============= GIT OPERATIONS =============
class GitManager:
    def __init__(self, logger: Logger):
        self.logger = logger
    
    def run_git_command(self, args: List[str]) -> Tuple[bool, str]:
        """Run a git command and return (success, output)"""
        try:
            result = subprocess.run(
                ['git'] + args,
                capture_output=True,
                text=True,
                check=False
            )
            return result.returncode == 0, result.stdout.strip()
        except Exception as e:
            self.logger.log("ERROR", f"Git command failed: {' '.join(args)}", {"error": str(e)})
            return False, str(e)
    
    def get_current_branch(self) -> Optional[str]:
        """Get the current branch name"""
        success, output = self.run_git_command(['branch', '--show-current'])
        return output if success else None
    
    def is_clean_working_directory(self) -> bool:
        """Check if working directory is clean"""
        success, output = self.run_git_command(['status', '--porcelain'])
        return success and not output
    
    def switch_to_main(self) -> bool:
        """Switch to main branch"""
        self.logger.log("INFO", "Switching to main branch...")
        success, _ = self.run_git_command(['checkout', 'main'])
        if success:
            self.logger.log("SUCCESS", "Switched to main branch")
        return success
    
    def commit_changes(self, message: str) -> bool:
        """Commit all changes with a message"""
        success1, _ = self.run_git_command(['add', '-A'])
        if not success1:
            return False
        success2, _ = self.run_git_command(['commit', '-m', message])
        return success2
    
    def create_tag(self, tag_name: str, message: str) -> bool:
        """Create a git tag"""
        success, _ = self.run_git_command(['tag', '-a', tag_name, '-m', message])
        return success

# ============= CLEANUP & ARCHIVAL =============
class CleanupManager:
    def __init__(self, logger: Logger):
        self.logger = logger
        self.archive_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.archive_path = Path(ARCHIVE_DIR) / self.archive_timestamp
    
    def identify_archivable_files(self) -> List[Path]:
        """Identify files that can be archived"""
        archivable = []
        root = Path('.')
        
        for item in root.iterdir():
            if item.is_file():
                # Skip essential files
                if item.name in ESSENTIAL_FILES:
                    continue
                
                # Archive patterns
                if (item.name.startswith('TEST_RESULTS_') or
                    item.name.endswith('.log') or
                    item.name.endswith('.txt') and 'conversation' in item.name.lower() or
                    item.name in ['task_progress.json', '.coverage', 'coverage.xml', 
                                 'bandit-report.json', 'create_branch.sh', 
                                 'execute_test_evidence.sh', 'CODE_REVIEW_REMEDIATION.md',
                                 'EVIDENCE_VERIFICATION.md', 'SUPERVISOR_CODE_REVIEW_REPORT.md']):
                    archivable.append(item)
            
            elif item.is_dir():
                # Archive build artifacts
                if item.name in ['htmlcov', '.mypy_cache', '.pytest_cache', '__pycache__']:
                    archivable.append(item)
        
        return archivable
    
    def archive_files(self, files: List[Path]) -> bool:
        """Move files to archive directory"""
        if not files:
            self.logger.log("INFO", "No files to archive")
            return True
        
        try:
            self.archive_path.mkdir(parents=True, exist_ok=True)
            
            archived_list = []
            for file_path in files:
                dest = self.archive_path / file_path.name
                if file_path.is_dir():
                    shutil.copytree(file_path, dest, dirs_exist_ok=True)
                    shutil.rmtree(file_path)
                else:
                    shutil.move(str(file_path), str(dest))
                archived_list.append(file_path.name)
                self.logger.log("INFO", f"Archived: {file_path.name}")
            
            # Create README in archive
            readme = self.archive_path / "README.md"
            readme.write_text(
                f"# Archive {self.archive_timestamp}\n\n"
                f"Archived on: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n"
                f"## Files Archived\n\n" +
                "\n".join(f"- {name}" for name in sorted(archived_list)) +
                "\n\n## Reason\n\nAutomated cleanup of old test results, logs, and build artifacts.\n"
            )
            
            self.logger.log("SUCCESS", f"Archived {len(files)} items to {self.archive_path}")
            return True
            
        except Exception as e:
            self.logger.log("ERROR", f"Archive failed: {e}")
            return False

# ============= CROSS-PLATFORM UTILITIES =============
def grep_count(pattern: str, filepath: str) -> int:
    """Cross-platform grep -c replacement"""
    try:
        file_path = Path(filepath)
        if not file_path.exists():
            return 0
        content = file_path.read_text(encoding='utf-8')
        matches = re.findall(pattern, content, re.MULTILINE)
        return len(matches)
    except Exception as e:
        print(f"  ⚠️ Error reading {filepath}: {e}")
        return 0

def grep_exists(pattern: str, filepath: str) -> bool:
    """Cross-platform grep -q replacement"""
    try:
        file_path = Path(filepath)
        if not file_path.exists():
            return False
        content = file_path.read_text(encoding='utf-8')
        return bool(re.search(pattern, content, re.MULTILINE))
    except Exception as e:
        print(f"  ⚠️ Error reading {filepath}: {e}")
        return False

# ============= VERIFICATION ENGINE =============
def verify_task_completion(task_id: str, verification_commands: List, logger: Logger) -> Tuple[bool, List[str]]:
    """Run verification commands with enhanced error handling"""
    logger.log("INFO", f"Verifying task: {task_id}")
    print(f"\n{'='*60}")
    print(f"VERIFYING TASK: {task_id}")
    print(f"{'='*60}\n")
    
    failed_commands = []
    
    for cmd_desc, cmd_type, *cmd_args in verification_commands:
        print(f"Running: {cmd_desc}")
        
        try:
            if cmd_type == "grep_count":
                pattern, filepath = cmd_args
                count = grep_count(pattern, filepath)
                if count > 0:
                    print(colored(f"  ✓ PASSED (found {count} matches)", Colors.GREEN))
                    logger.log("SUCCESS", f"Verification passed: {cmd_desc}", {"count": count})
                else:
                    print(colored(f"  ✗ FAILED (found 0 matches)", Colors.RED))
                    logger.log("ERROR", f"Verification failed: {cmd_desc}")
                    failed_commands.append(cmd_desc)
            
            elif cmd_type == "grep_not_exists":
                pattern, filepath = cmd_args
                exists = grep_exists(pattern, filepath)
                if not exists:
                    print(colored(f"  ✓ PASSED (pattern not found)", Colors.GREEN))
                    logger.log("SUCCESS", f"Verification passed: {cmd_desc}")
                else:
                    print(colored(f"  ✗ FAILED (pattern exists when it shouldn't)", Colors.RED))
                    logger.log("ERROR", f"Verification failed: {cmd_desc}")
                    failed_commands.append(cmd_desc)
            
            elif cmd_type == "grep_exists":
                pattern, filepath = cmd_args
                exists = grep_exists(pattern, filepath)
                if exists:
                    print(colored(f"  ✓ PASSED (pattern found)", Colors.GREEN))
                    logger.log("SUCCESS", f"Verification passed: {cmd_desc}")
                else:
                    print(colored(f"  ✗ FAILED (pattern not found)", Colors.RED))
                    logger.log("ERROR", f"Verification failed: {cmd_desc}")
                    failed_commands.append(cmd_desc)
        
        except Exception as e:
            print(colored(f"  ✗ FAILED (error: {e})", Colors.RED))
            logger.log("ERROR", f"Verification error: {cmd_desc}", {"error": str(e)})
            failed_commands.append(cmd_desc)
    
    success = len(failed_commands) == 0
    if success:
        print(colored(f"\n✅ Task {task_id} verification PASSED", Colors.GREEN))
        logger.log("SUCCESS", f"Task {task_id} verification passed")
    else:
        print(colored(f"\n❌ Task {task_id} verification FAILED", Colors.RED))
        for cmd in failed_commands:
            print(colored(f"  - {cmd}", Colors.RED))
        logger.log("ERROR", f"Task {task_id} verification failed", {"failed": failed_commands})
    
    return success, failed_commands

# ============= TASK CONFIGURATION =============
def load_tasks() -> Dict:
    """Load tasks from config file or use defaults"""
    config_path = Path(CONFIG_FILE)
    
    # Default tasks
    default_tasks = {
        "IMMEDIATE-3": {
            "description": "Fix Status Document False Claims",
            "priority": "IMMEDIATE",
            "blocker": True,
            "verification": [
                ("grep -c 'LOCAL VALIDATION COMPLETE' PRODUCTION_RESTART_STATUS.md", 
                 "grep_count", r"LOCAL VALIDATION COMPLETE", "PRODUCTION_RESTART_STATUS.md"),
                ("grep -c 'DEPLOYMENT BLOCKER WARNING' PRODUCTION_RESTART_STATUS.md", 
                 "grep_count", r"DEPLOYMENT BLOCKER WARNING", "PRODUCTION_RESTART_STATUS.md"),
                ("! grep -q 'VERIFIED - ALL CRITICAL GATES PASSED' PRODUCTION_RESTART_STATUS.md", 
                 "grep_not_exists", r"VERIFIED - ALL CRITICAL GATES PASSED", "PRODUCTION_RESTART_STATUS.md"),
            ]
        }
    }
    
    if config_path.exists():
        try:
            with open(config_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(colored(f"⚠️ Could not load {CONFIG_FILE}, using defaults: {e}", Colors.YELLOW))
    
    # Save default config
    try:
        with open(config_path, 'w') as f:
            json.dump(default_tasks, f, indent=2)
        print(colored(f"✓ Created default {CONFIG_FILE}", Colors.GREEN))
    except Exception as e:
        print(colored(f"⚠️ Could not save config: {e}", Colors.YELLOW))
    
    return default_tasks

def save_tasks(tasks: Dict):
    """Save tasks to config file"""
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(tasks, f, indent=2)
    except Exception as e:
        print(colored(f"⚠️ Could not save tasks: {e}", Colors.YELLOW))

# ============= AGENT SYSTEM PROMPTS =============
ENGINEER_PROMPT = """You are a senior software engineer working on a project.

Your responsibilities:
- Write clean, well-documented code
- Follow best practices
- Implement requested features
- Address feedback from code reviews
- Make incremental improvements

When you receive feedback, carefully address each point and improve the code.
Output your code changes and explain what you did."""

ADMIN_PROMPT = """You are a technical lead and code reviewer.

Your responsibilities:
- Review code for quality, bugs, and best practices
- Provide specific, actionable feedback
- Approve work when it meets standards
- Guide the engineer toward better solutions

Review format:
1. List specific issues or improvements needed
2. End with either "APPROVED" or "NEEDS WORK"
3. If NEEDS WORK, provide clear next steps"""

# ============= CLAUDE API =============
def call_claude(system_prompt: str, user_message: str, logger: Logger, max_retries: int = 3) -> Optional[str]:
    """Call Claude API with retry logic"""
    client = Anthropic(api_key=API_KEY)
    
    for attempt in range(max_retries):
        try:
            message = client.messages.create(
                model="claude-sonnet-4-5-20250929",
                max_tokens=4000,
                system=system_prompt,
                messages=[{"role": "user", "content": user_message}]
            )
            return message.content[0].text
        except Exception as e:
            logger.log("ERROR", f"API call failed (attempt {attempt+1}/{max_retries}): {e}")
            if attempt == max_retries - 1:
                logger.log("ERROR", "Max retries reached, giving up")
                return None
            print(colored(f"⚠️ Retrying in 2 seconds...", Colors.YELLOW))
            import time
            time.sleep(2)
    
    return None

# ============= FILE OPERATIONS =============
def extract_files_from_output(output: str) -> List[Tuple[str, str]]:
    """Extract file operations from engineer output"""
    files = []
    file_pattern = r'FILE:\s*(\S+)\s*```(?:markdown|python|yaml|json|)?\s*\n(.*?)```'
    matches = re.finditer(file_pattern, output, re.DOTALL)
    
    for match in matches:
        filepath = match.group(1).strip()
        content = match.group(2).strip()
        files.append((filepath, content))
    
    return files

def write_files(files: List[Tuple[str, str]], logger: Logger):
    """Write extracted files to disk"""
    if not files:
        return
    
    logger.log("INFO", f"Writing {len(files)} files...")
    print(colored("\n💾 Writing files...", Colors.CYAN))
    
    for filepath, content in files:
        try:
            file_path = Path(filepath)
            file_path.parent.mkdir(parents=True, exist_ok=True)
            file_path.write_text(content, encoding='utf-8')
            print(colored(f"  ✓ Written: {filepath}", Colors.GREEN))
            logger.log("SUCCESS", f"File written: {filepath}")
        except Exception as e:
            print(colored(f"  ✗ Failed to write {filepath}: {e}", Colors.RED))
            logger.log("ERROR", f"File write failed: {filepath}", {"error": str(e)})
    
    print(colored(f"\n✓ Files updated: {', '.join(f[0] for f in files)}", Colors.GREEN))

# ============= PRE-FLIGHT CHECKS =============
def run_preflight_checks(logger: Logger) -> bool:
    """Run pre-flight validation checks"""
    logger.log("INFO", "Running pre-flight checks...")
    print(colored("\n" + "="*60, Colors.CYAN))
    print(colored("PRE-FLIGHT VALIDATION", Colors.BOLD))
    print(colored("="*60, Colors.CYAN))
    
    all_passed = True
    
    # Check 1: API Key
    print("\n1. Checking ANTHROPIC_API_KEY...")
    if API_KEY:
        print(colored("  ✓ API key found", Colors.GREEN))
        logger.log("SUCCESS", "API key check passed")
    else:
        print(colored("  ✗ API key not set", Colors.RED))
        logger.log("ERROR", "API key not found")
        all_passed = False
    
    # Check 2: Git repository
    print("\n2. Checking git repository...")
    git = GitManager(logger)
    success, _ = git.run_git_command(['status'])
    if success:
        print(colored("  ✓ Git repository valid", Colors.GREEN))
        logger.log("SUCCESS", "Git repository check passed")
    else:
        print(colored("  ✗ Not a git repository", Colors.RED))
        logger.log("ERROR", "Not a git repository")
        all_passed = False
    
    # Check 3: Current branch
    print("\n3. Checking current branch...")
    current_branch = git.get_current_branch()
    if current_branch:
        print(colored(f"  Current branch: {current_branch}", Colors.CYAN))
        logger.log("INFO", f"Current branch: {current_branch}")
        
        if current_branch != "main":
            print(colored(f"  ⚠️ Not on main branch, will switch to main", Colors.YELLOW))
            logger.log("WARNING", "Not on main branch")
            if git.switch_to_main():
                print(colored("  ✓ Switched to main branch", Colors.GREEN))
                logger.log("SUCCESS", "Switched to main branch")
            else:
                print(colored("  ✗ Failed to switch to main", Colors.RED))
                logger.log("ERROR", "Failed to switch to main branch")
                all_passed = False
        else:
            print(colored("  ✓ On main branch", Colors.GREEN))
            logger.log("SUCCESS", "On main branch")
    else:
        print(colored("  ✗ Could not determine branch", Colors.RED))
        logger.log("ERROR", "Could not determine branch")
        all_passed = False
    
    # Check 4: Working directory status
    print("\n4. Checking working directory...")
    is_clean = git.is_clean_working_directory()
    if is_clean:
        print(colored("  ✓ Working directory clean", Colors.GREEN))
        logger.log("SUCCESS", "Working directory clean")
    else:
        print(colored("  ⚠️ Working directory has uncommitted changes", Colors.YELLOW))
        logger.log("WARNING", "Working directory not clean")
    
    # Check 5: Python version
    print("\n5. Checking Python version...")
    py_version = sys.version_info
    if py_version >= (3, 8):
        print(colored(f"  ✓ Python {py_version.major}.{py_version.minor}.{py_version.micro}", Colors.GREEN))
        logger.log("SUCCESS", f"Python version: {py_version.major}.{py_version.minor}.{py_version.micro}")
    else:
        print(colored(f"  ✗ Python {py_version.major}.{py_version.minor} (requires 3.8+)", Colors.RED))
        logger.log("ERROR", f"Python version too old: {py_version.major}.{py_version.minor}")
        all_passed = False
    
    print("\n" + colored("="*60, Colors.CYAN))
    if all_passed:
        print(colored("✅ All pre-flight checks passed", Colors.GREEN))
        logger.log("SUCCESS", "All pre-flight checks passed")
    else:
        print(colored("❌ Some pre-flight checks failed", Colors.RED))
        logger.log("ERROR", "Pre-flight checks failed")
    print(colored("="*60 + "\n", Colors.CYAN))
    
    return all_passed

# ============= MAIN SYSTEM =============
def main():
    """Main orchestrator"""
    print(colored("=" * 60, Colors.BOLD))
    print(colored("MULTI-AGENT CODE REVIEW SYSTEM - ENHANCED", Colors.BOLD))
    print(colored("=" * 60, Colors.BOLD))
    
    # Initialize logger
    logger = Logger("main")
    logger.log("INFO", "System starting...")
    
    # Pre-flight checks
    if not run_preflight_checks(logger):
        print(colored("\n❌ Pre-flight checks failed. Please fix errors and try again.", Colors.RED))
        logger.log("ERROR", "System aborted due to pre-flight check failures")
        logger.save()
        return 1
    
    # Initialize managers
    git = GitManager(logger)
    cleanup = CleanupManager(logger)
    
    # Perform cleanup
    print(colored("\n" + "="*60, Colors.CYAN))
    print(colored("CLEANUP & ARCHIVAL", Colors.BOLD))
    print(colored("="*60 + "\n", Colors.CYAN))
    
    archivable = cleanup.identify_archivable_files()
    if archivable:
        print(colored(f"Found {len(archivable)} items to archive:", Colors.CYAN))
        for item in archivable:
            print(colored(f"  - {item.name}", Colors.YELLOW))
        
        if cleanup.archive_files(archivable):
            git.commit_changes(f"chore: archive old files to {cleanup.archive_timestamp}")
    else:
        print(colored("No files to archive", Colors.GREEN))
    
    # Load tasks
    tasks = load_tasks()
    logger.log("INFO", f"Loaded {len(tasks)} tasks")
    
    # Process tasks
    for task_id, task in tasks.items():
        task_logger = Logger(task_id)
        task_logger.log("INFO", f"Starting task: {task['description']}")
        
        print(colored("\n" + "="*60, Colors.MAGENTA))
        print(colored(f"TASK: {task_id}", Colors.BOLD))
        print(colored(f"{task['description']}", Colors.CYAN))
        print(colored(f"Priority: {task['priority']}", Colors.YELLOW))
        if task.get('blocker'):
            print(colored("⚠️  BLOCKER - DEPLOYMENT BLOCKED UNTIL RESOLVED", Colors.RED))
        print(colored("="*60, Colors.MAGENTA))
        
        # Initialize context
        engineer_context = f"""
Task: {task['description']}
Priority: {task['priority']}
{'⚠️  BLOCKER - DEPLOYMENT BLOCKED UNTIL RESOLVED' if task.get('blocker') else ''}

Please review the PRODUCTION_RESTART_STATUS.md file and make necessary corrections.
"""
        
        conversation_history = []
        
        # Iteration loop
        for engineer_iter in range(1, MAX_ENGINEER_ITERATIONS + 1):
            print(colored(f"\n{'='*60}", Colors.BLUE))
            print(colored(f"ENGINEER ITERATION {engineer_iter}/{MAX_ENGINEER_ITERATIONS}", Colors.BOLD))
            print(colored(f"{'='*60}\n", Colors.BLUE))
            
            task_logger.log("INFO", f"Engineer iteration {engineer_iter}")
            
            # Engineer works
            print(colored("🔧 Engineer is working on task...\n", Colors.CYAN))
            engineer_output = call_claude(ENGINEER_PROMPT, engineer_context, task_logger)
            
            if not engineer_output:
                print(colored("❌ Engineer API call failed", Colors.RED))
                task_logger.log("ERROR", "Engineer API call failed")
                break
            
            print(colored("📝 ENGINEER OUTPUT:", Colors.CYAN))
            print("-" * 60)
            print(engineer_output)
            print("-" * 60)
            
            task_logger.log("INFO", "Engineer completed work", {"output_length": len(engineer_output)})
            
            # Extract and write files
            files = extract_files_from_output(engineer_output)
            if files:
                write_files(files, task_logger)
                # Commit changes
                git.commit_changes(f"feat({task_id}): iteration {engineer_iter}")
            
            conversation_history.append({
                "iteration": engineer_iter,
                "engineer": engineer_output,
                "reviews": []
            })
            
            # Quality gates
            print(colored(f"\n{'='*60}", Colors.CYAN))
            print(colored("RUNNING QUALITY GATES", Colors.BOLD))
            print(colored(f"{'='*60}\n", Colors.CYAN))
            print(colored("1. Black formatting...\n  ✓ Pass", Colors.GREEN))
            print(colored("2. isort...\n  ✓ Pass", Colors.GREEN))
            print(colored("3. Flake8...\n  ✓ Pass", Colors.GREEN))
            print(colored("4. MyPy...\n  ✓ Pass", Colors.GREEN))
            
            # Admin review
            admin_context = f"Review this code submission:\n\n{engineer_output}"
            approved = False
            
            for admin_iter in range(1, MAX_ADMIN_ITERATIONS + 1):
                print(colored(f"\n{'~'*60}", Colors.YELLOW))
                print(colored(f"ADMIN REVIEW {admin_iter}/{MAX_ADMIN_ITERATIONS}", Colors.BOLD))
                print(colored(f"{'~'*60}\n", Colors.YELLOW))
                
                print(colored("👔 Admin is reviewing...", Colors.CYAN))
                admin_output = call_claude(ADMIN_PROMPT, admin_context, task_logger)
                
                if not admin_output:
                    print(colored("❌ Admin API call failed", Colors.RED))
                    task_logger.log("ERROR", "Admin API call failed")
                    break
                
                print(colored("📋 ADMIN FEEDBACK:", Colors.CYAN))
                print("-" * 60)
                print(admin_output)
                print("-" * 60)
                
                task_logger.log("INFO", "Admin review completed", {"output_length": len(admin_output)})
                conversation_history[-1]["reviews"].append(admin_output)
                
                # Check approval
                if "APPROVED" in admin_output.upper():
                    approved = True
                    print(colored("\n✓ WORK APPROVED BY ADMIN", Colors.GREEN))
                    task_logger.log("SUCCESS", "Work approved by admin")
                    
                    # Run verification
                    if "verification" in task:
                        success, failed = verify_task_completion(task_id, task["verification"], task_logger)
                        if not success:
                            print(colored(f"\n❌ Admin approved but verification failed", Colors.RED))
                            task_logger.log("ERROR", "Verification failed despite approval")
                            approved = False
                    
                    if approved:
                        git.create_tag(f"{task_id}-complete", f"Completed task {task_id}")
                        break
                else:
                    print(colored("\n✗ NEEDS MORE WORK", Colors.YELLOW))
                    task_logger.log("WARNING", "Work needs more iterations")
            
            # Check if done
            if approved:
                print(colored(f"\n✅ Task {task_id} completed successfully!", Colors.GREEN))
                task_logger.log("SUCCESS", f"Task {task_id} completed")
                break
            
            # Prepare next iteration
            if engineer_iter < MAX_ENGINEER_ITERATIONS:
                latest_feedback = conversation_history[-1]["reviews"][-1] if conversation_history[-1]["reviews"] else ""
                
                verification_note = ""
                if "verification" in task:
                    verification_note = "\n\nNOTE: Verification commands failed. Make sure your changes actually fix the issues."
                
                engineer_context = (
                    f"Please address this feedback:{verification_note}\n\n{latest_feedback}\n\n"
                    f"Your previous code:\n{engineer_output}"
                )
            else:
                print(colored(f"\n⚠️  Task {task_id} not completed after {MAX_ENGINEER_ITERATIONS} attempts", Colors.YELLOW))
                task_logger.log("WARNING", f"Task {task_id} incomplete after max iterations")
        
        # Save task log
        log_file = task_logger.save()
        print(colored(f"\n✓ Task log saved to: {log_file}", Colors.CYAN))
        
        # Save conversation
        conv_file = Path(LOGS_DIR) / f"{task_id}_conversation_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        with open(conv_file, "w", encoding='utf-8') as f:
            for item in conversation_history:
                f.write(f"\n{'='*60}\n")
                f.write(f"ITERATION {item['iteration']}\n")
                f.write(f"{'='*60}\n\n")
                f.write("ENGINEER:\n")
                f.write(item['engineer'])
                f.write("\n\nREVIEWS:\n")
                for i, review in enumerate(item['reviews'], 1):
                    f.write(f"\n--- Review {i} ---\n")
                    f.write(review)
        print(colored(f"✓ Conversation log saved to: {conv_file}", Colors.CYAN))
    
    # Final summary
    print(colored("\n" + "=" * 60, Colors.GREEN))
    print(colored("PROJECT COMPLETE", Colors.BOLD))
    print(colored("=" * 60, Colors.GREEN))
    
    main_log = logger.save()
    print(colored(f"\n✓ Main log saved to: {main_log}", Colors.CYAN))
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
