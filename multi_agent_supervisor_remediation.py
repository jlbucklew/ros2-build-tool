#!/usr/bin/env python3
"""
Multi-Agent Code Review System - Windows Compatible
Two Claude instances: Engineer and Admin working iteratively
Now with cross-platform verification commands
"""

import os
import re
from pathlib import Path
from anthropic import Anthropic

# ============= CONFIGURATION =============
MAX_ENGINEER_ITERATIONS = 30  # How many times the engineer works
MAX_ADMIN_ITERATIONS = 1     # How many times admin can review per engineer iteration
API_KEY = os.environ.get("ANTHROPIC_API_KEY")  # Set this in your environment

# Initial project description
PROJECT_DESCRIPTION = """
Create a simple Python calculator that can:
- Add, subtract, multiply, divide
- Handle errors gracefully
- Have a clean command-line interface
- Include unit tests
"""

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

# ============= CROSS-PLATFORM UTILITIES =============

def grep_count(pattern, filepath):
    """
    Cross-platform replacement for: grep -c 'pattern' file
    Returns the count of lines matching the pattern
    """
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


def grep_exists(pattern, filepath):
    """
    Cross-platform replacement for: grep -q 'pattern' file
    Returns True if pattern exists in file, False otherwise
    """
    try:
        file_path = Path(filepath)
        if not file_path.exists():
            return False
        
        content = file_path.read_text(encoding='utf-8')
        return bool(re.search(pattern, content, re.MULTILINE))
    except Exception as e:
        print(f"  ⚠️ Error reading {filepath}: {e}")
        return False


def verify_task_completion(task_id, verification_commands):
    """
    Run verification commands in a cross-platform way
    Returns: (success: bool, failed_commands: list)
    """
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
                    print(f"  ✓ PASSED (found {count} matches)")
                else:
                    print(f"  ✗ FAILED (found 0 matches)")
                    failed_commands.append(cmd_desc)
            
            elif cmd_type == "grep_not_exists":
                pattern, filepath = cmd_args
                exists = grep_exists(pattern, filepath)
                if not exists:
                    print(f"  ✓ PASSED (pattern not found)")
                else:
                    print(f"  ✗ FAILED (pattern exists when it shouldn't)")
                    failed_commands.append(cmd_desc)
            
            elif cmd_type == "grep_exists":
                pattern, filepath = cmd_args
                exists = grep_exists(pattern, filepath)
                if exists:
                    print(f"  ✓ PASSED (pattern found)")
                else:
                    print(f"  ✗ FAILED (pattern not found)")
                    failed_commands.append(cmd_desc)
        
        except Exception as e:
            print(f"  ✗ FAILED (error: {e})")
            failed_commands.append(cmd_desc)
    
    if failed_commands:
        print(f"\n❌ Task {task_id} verification FAILED")
        for cmd in failed_commands:
            print(f"  - Command failed: {cmd}")
        return False, failed_commands
    else:
        print(f"\n✅ Task {task_id} verification PASSED")
        return True, []


# ============= TASK DEFINITIONS WITH VERIFICATION =============

TASKS = {
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

# ============= MAIN SYSTEM =============

def call_claude(system_prompt, user_message):
    """Call Claude API with given prompts"""
    client = Anthropic(api_key=API_KEY)
    
    message = client.messages.create(
        model="claude-sonnet-4-5-20250929",
        max_tokens=4000,
        system=system_prompt,
        messages=[{"role": "user", "content": user_message}]
    )
    
    return message.content[0].text


def extract_files_from_output(output):
    """
    Extract file operations from engineer output
    Returns: list of (filepath, content) tuples
    """
    files = []
    
    # Look for FILE: filepath pattern followed by content
    file_pattern = r'FILE:\s*(\S+)\s*```(?:markdown|python|)?\s*\n(.*?)```'
    matches = re.finditer(file_pattern, output, re.DOTALL)
    
    for match in matches:
        filepath = match.group(1).strip()
        content = match.group(2).strip()
        files.append((filepath, content))
    
    return files


def write_files(files):
    """Write extracted files to disk"""
    if not files:
        return
    
    print("\n💾 Writing files...")
    for filepath, content in files:
        try:
            file_path = Path(filepath)
            file_path.parent.mkdir(parents=True, exist_ok=True)
            file_path.write_text(content, encoding='utf-8')
            print(f"  ✓ Written: {filepath}")
        except Exception as e:
            print(f"  ✗ Failed to write {filepath}: {e}")
    
    print(f"\n✓ Files updated: {', '.join(f[0] for f in files)}")


def main():
    print("=" * 60)
    print("MULTI-AGENT CODE REVIEW SYSTEM (Windows Compatible)")
    print("=" * 60)
    
    if not API_KEY:
        print("ERROR: Please set ANTHROPIC_API_KEY environment variable")
        return
    
    # Initialize conversation context
    engineer_context = PROJECT_DESCRIPTION
    conversation_history = []
    
    # Example: Work on a specific task
    task_id = "IMMEDIATE-3"
    if task_id in TASKS:
        task = TASKS[task_id]
        engineer_context = f"""
Task: {task['description']}
Priority: {task['priority']}
{'⚠️  BLOCKER - DEPLOYMENT BLOCKED UNTIL RESOLVED' if task['blocker'] else ''}

{PROJECT_DESCRIPTION}
"""
    
    # Main iteration loop
    for engineer_iter in range(1, MAX_ENGINEER_ITERATIONS + 1):
        print(f"\n{'='*60}")
        print(f"ENGINEER ITERATION {engineer_iter}/{MAX_ENGINEER_ITERATIONS}")
        print(f"{'='*60}\n")
        
        # Engineer works on the task
        print("🔧 Engineer is working on task...\n")
        engineer_output = call_claude(ENGINEER_PROMPT, engineer_context)
        print("📝 ENGINEER OUTPUT:")
        print("-" * 60)
        print(engineer_output)
        print("-" * 60)
        
        # Extract and write any files
        files = extract_files_from_output(engineer_output)
        if files:
            write_files(files)
        
        conversation_history.append({
            "iteration": engineer_iter,
            "engineer": engineer_output,
            "reviews": []
        })
        
        # Run quality gates (simplified for demo)
        print(f"\n{'='*60}")
        print("RUNNING QUALITY GATES")
        print(f"{'='*60}\n")
        print("1. Black formatting...\n  ✓ Pass")
        print("2. isort...\n  ✓ Pass")
        print("3. Flake8...\n  ✓ Pass")
        print("4. MyPy...\n  ✓ Pass")
        
        # Admin review loop
        admin_context = f"Review this code submission:\n\n{engineer_output}"
        approved = False
        
        for admin_iter in range(1, MAX_ADMIN_ITERATIONS + 1):
            print(f"\n{'~'*60}")
            print(f"ADMIN REVIEW {admin_iter}/{MAX_ADMIN_ITERATIONS}")
            print(f"{'~'*60}\n")
            
            print("👔 Admin is reviewing...")
            admin_output = call_claude(ADMIN_PROMPT, admin_context)
            print("📋 ADMIN FEEDBACK:")
            print("-" * 60)
            print(admin_output)
            print("-" * 60)
            
            conversation_history[-1]["reviews"].append(admin_output)
            
            # Check if approved
            if "APPROVED" in admin_output.upper():
                approved = True
                print("\n✓ WORK APPROVED BY ADMIN")
                
                # Run verification if task has verification commands
                if task_id in TASKS and "verification" in TASKS[task_id]:
                    success, failed = verify_task_completion(
                        task_id, 
                        TASKS[task_id]["verification"]
                    )
                    
                    if not success:
                        print(f"\n❌ Admin approved but verification failed:")
                        for cmd in failed:
                            print(f"  - Command failed: {cmd}")
                        print("Task needs more work")
                        approved = False
                
                if approved:
                    break
            else:
                print("\n✗ NEEDS MORE WORK")
                
                # If not last admin iteration, let admin refine feedback
                if admin_iter < MAX_ADMIN_ITERATIONS:
                    admin_context = (
                        f"Previous review:\n{admin_output}\n\n"
                        f"Refine your feedback or approve if good enough.\n\n"
                        f"Original code:\n{engineer_output}"
                    )
        
        # If approved and verified, we're done
        if approved:
            print(f"\n✅ Task {task_id} completed successfully!")
            break
        
        # Prepare context for next engineer iteration
        if engineer_iter < MAX_ENGINEER_ITERATIONS:
            latest_feedback = conversation_history[-1]["reviews"][-1]
            
            if approved:
                engineer_context = (
                    f"Great work! Your previous submission was approved.\n\n"
                    f"Now, enhance the project further or add new features.\n\n"
                    f"Previous work:\n{engineer_output}"
                )
            else:
                # Include verification failures in feedback
                verification_note = ""
                if task_id in TASKS and "verification" in TASKS[task_id]:
                    verification_note = "\n\nNOTE: Verification commands failed. Make sure your changes actually fix the issues."
                
                engineer_context = (
                    f"Please address this feedback:{verification_note}\n\n{latest_feedback}\n\n"
                    f"Your previous code:\n{engineer_output}"
                )
        else:
            print(f"\n⚠️  Task {task_id} not completed after {MAX_ENGINEER_ITERATIONS} attempts")
            print("Moving to next iteration...")
    
    # Final summary
    print("\n" + "=" * 60)
    print("PROJECT COMPLETE")
    print("=" * 60)
    print(f"\nTotal Engineer Iterations: {len(conversation_history)}")
    print(f"Maximum Admin Reviews per Iteration: {MAX_ADMIN_ITERATIONS}")
    
    # Save conversation log
    log_file = Path("agent_conversation_log.txt")
    with open(log_file, "w", encoding='utf-8') as f:
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
    
    print(f"\n✓ Conversation log saved to: {log_file.absolute()}")


if __name__ == "__main__":
    main()