#!/bin/bash

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored messages
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
usage() {
    cat << EOF
Usage: $0 <source_directory> [options]

This script compares the current directory with a source directory, applies changes,
and creates a GitLab merge request.

Arguments:
    source_directory    Path to the directory containing the changes to apply

Options:
    -b, --branch NAME       Branch name for the merge request (default: sync-<timestamp>)
    -t, --title TITLE       Title for the merge request (default: "Sync changes from source")
    -d, --description DESC  Description for the merge request
    -m, --target-branch     Target branch for merge request (default: current branch)
    -r, --remote REMOTE     Git remote name (default: origin)
    -p, --project-id ID     GitLab project ID (optional, will try to detect)
    -u, --gitlab-url URL    GitLab instance URL (optional, will try to detect)
    -n, --dry-run           Show what would be done without making changes
    -h, --help              Show this help message

Environment Variables:
    GITLAB_TOKEN           Personal access token for GitLab API (required)
    GITLAB_URL             GitLab instance URL (optional, will try to detect from git remote)
    GITLAB_PROJECT_ID      GitLab project ID (optional, will try to detect)

Examples:
    $0 /path/to/source/directory
    $0 /path/to/source/directory -b my-sync-branch -t "Update from upstream"
    $0 /path/to/source/directory --dry-run
EOF
    exit 1
}

# Parse arguments
SOURCE_DIR=""
BRANCH_NAME=""
MR_TITLE="Sync changes from source"
MR_DESCRIPTION=""
TARGET_BRANCH=""
REMOTE="origin"
PROJECT_ID=""
GITLAB_URL=""
DRY_RUN=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--branch)
            BRANCH_NAME="$2"
            shift 2
            ;;
        -t|--title)
            MR_TITLE="$2"
            shift 2
            ;;
        -d|--description)
            MR_DESCRIPTION="$2"
            shift 2
            ;;
        -m|--target-branch)
            TARGET_BRANCH="$2"
            shift 2
            ;;
        -r|--remote)
            REMOTE="$2"
            shift 2
            ;;
        -p|--project-id)
            PROJECT_ID="$2"
            shift 2
            ;;
        -u|--gitlab-url)
            GITLAB_URL="$2"
            shift 2
            ;;
        -n|--dry-run)
            DRY_RUN=true
            shift
            ;;
        -h|--help)
            usage
            ;;
        -*)
            print_error "Unknown option: $1"
            usage
            ;;
        *)
            if [[ -z "$SOURCE_DIR" ]]; then
                SOURCE_DIR="$1"
            else
                print_error "Unexpected argument: $1"
                usage
            fi
            shift
            ;;
    esac
done

# Check if source directory is provided
if [[ -z "$SOURCE_DIR" ]]; then
    print_error "Source directory is required"
    usage
fi

# Validate source directory
if [[ ! -d "$SOURCE_DIR" ]]; then
    print_error "Source directory does not exist: $SOURCE_DIR"
    exit 1
fi

# Get absolute paths
CURRENT_DIR=$(pwd)
SOURCE_DIR=$(cd "$SOURCE_DIR" && pwd)

print_info "Current directory: $CURRENT_DIR"
print_info "Source directory: $SOURCE_DIR"

# Check if current directory is a git repository
if ! git rev-parse --git-dir > /dev/null 2>&1; then
    print_error "Current directory is not a git repository"
    exit 1
fi

# Check for uncommitted changes
if [[ -n "$(git status --porcelain)" ]]; then
    print_warning "You have uncommitted changes. They will be stashed."
    if [[ "$DRY_RUN" == false ]]; then
        read -p "Continue? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
        git stash push -m "Stashed before sync: $(date)"
    fi
fi

# Get current branch
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
if [[ -z "$TARGET_BRANCH" ]]; then
    TARGET_BRANCH="$CURRENT_BRANCH"
fi

# Generate branch name if not provided
if [[ -z "$BRANCH_NAME" ]]; then
    TIMESTAMP=$(date +%Y%m%d-%H%M%S)
    BRANCH_NAME="sync-${TIMESTAMP}"
fi

print_info "Target branch: $TARGET_BRANCH"
print_info "New branch name: $BRANCH_NAME"

# Check for differences
print_info "Checking differences between directories..."
DIFF_OUTPUT=$(diff -rq --exclude='.git' "$CURRENT_DIR" "$SOURCE_DIR" 2>&1 || true)

if [[ -z "$DIFF_OUTPUT" ]]; then
    print_success "No differences found between directories"
    exit 0
fi

# Show summary of differences
print_info "Found differences:"
echo "$DIFF_OUTPUT" | head -20
if [[ $(echo "$DIFF_OUTPUT" | wc -l) -gt 20 ]]; then
    print_warning "... and more (showing first 20 lines)"
fi

if [[ "$DRY_RUN" == true ]]; then
    print_warning "DRY RUN MODE - No changes will be made"
    print_info "Would create branch: $BRANCH_NAME"
    print_info "Would apply changes from: $SOURCE_DIR"
    print_info "Would create merge request with title: $MR_TITLE"
    exit 0
fi

# Create and checkout new branch
print_info "Creating branch: $BRANCH_NAME"
if git show-ref --verify --quiet refs/heads/"$BRANCH_NAME"; then
    print_warning "Branch $BRANCH_NAME already exists. Checking it out..."
    git checkout "$BRANCH_NAME"
else
    git checkout -b "$BRANCH_NAME" "$TARGET_BRANCH"
fi

# Function to copy files preserving directory structure
copy_changes() {
    local src="$1"
    local dst="$2"
    
    # Use rsync if available, otherwise use cp
    if command -v rsync > /dev/null 2>&1; then
        rsync -av --exclude='.git' "$src/" "$dst/"
    else
        # Fallback: use find and cp
        find "$src" -type f ! -path '*/.git/*' -exec sh -c '
            for file; do
                rel_path="${file#$1/}"
                dest="$2/$rel_path"
                mkdir -p "$(dirname "$dest")"
                cp "$file" "$dest"
            done
        ' _ "$src" "$dst" {} +
    fi
}

# Apply changes
print_info "Applying changes from source directory..."
copy_changes "$SOURCE_DIR" "$CURRENT_DIR"

# Check what files were changed
CHANGED_FILES=$(git status --porcelain | awk '{print $2}')

if [[ -z "$CHANGED_FILES" ]]; then
    print_warning "No file changes detected after copying. The directories may be identical."
    git checkout "$CURRENT_BRANCH"
    if git show-ref --verify --quiet refs/heads/"$BRANCH_NAME"; then
        git branch -D "$BRANCH_NAME"
    fi
    exit 0
fi

print_info "Changed files:"
echo "$CHANGED_FILES" | head -10
if [[ $(echo "$CHANGED_FILES" | wc -l) -gt 10 ]]; then
    print_warning "... and more files"
fi

# Stage all changes
# print_info "Staging changes..."
# git add -A

# # Create commit
# COMMIT_MSG="Sync changes from $SOURCE_DIR

# Applied changes from source directory: $SOURCE_DIR
# Generated by sync script on $(date)"

# print_info "Creating commit..."
# git commit -m "$COMMIT_MSG" || {
#     print_error "Failed to create commit. No changes to commit?"
#     git checkout "$CURRENT_BRANCH"
#     exit 1
# }

# # Push branch
# print_info "Pushing branch to remote..."
# git push -u "$REMOTE" "$BRANCH_NAME" || {
#     print_error "Failed to push branch to remote"
#     exit 1
# }

# # Detect GitLab URL and project if not provided
# if [[ -z "$GITLAB_URL" ]] || [[ -z "$PROJECT_ID" ]]; then
#     print_info "Detecting GitLab configuration..."
#     REMOTE_URL=$(git remote get-url "$REMOTE" 2>/dev/null || echo "")
    
#     if [[ -n "$REMOTE_URL" ]]; then
#         # Try to extract GitLab URL from remote
#         if [[ "$REMOTE_URL" =~ git@([^:]+):(.+)\.git ]]; then
#             GITLAB_HOST="${BASH_REMATCH[1]}"
#             PROJECT_PATH="${BASH_REMATCH[2]}"
#             if [[ -z "$GITLAB_URL" ]]; then
#                 GITLAB_URL="https://${GITLAB_HOST}"
#             fi
#             if [[ -z "$PROJECT_ID" ]]; then
#                 # Try to get project ID from API
#                 if [[ -n "${GITLAB_TOKEN:-}" ]]; then
#                     PROJECT_ID=$(curl -s --header "PRIVATE-TOKEN: $GITLAB_TOKEN" \
#                         "${GITLAB_URL}/api/v4/projects/${PROJECT_PATH//\//%2F}" \
#                         2>/dev/null | grep -o '"id":[0-9]*' | head -1 | cut -d: -f2 || echo "")
#                 fi
#             fi
#         elif [[ "$REMOTE_URL" =~ https://([^/]+)/(.+)\.git ]]; then
#             GITLAB_HOST="${BASH_REMATCH[1]}"
#             PROJECT_PATH="${BASH_REMATCH[2]}"
#             if [[ -z "$GITLAB_URL" ]]; then
#                 GITLAB_URL="https://${GITLAB_HOST}"
#             fi
#             if [[ -z "$PROJECT_ID" ]]; then
#                 if [[ -n "${GITLAB_TOKEN:-}" ]]; then
#                     PROJECT_ID=$(curl -s --header "PRIVATE-TOKEN: $GITLAB_TOKEN" \
#                         "${GITLAB_URL}/api/v4/projects/${PROJECT_PATH//\//%2F}" \
#                         2>/dev/null | grep -o '"id":[0-9]*' | head -1 | cut -d: -f2 || echo "")
#                 fi
#             fi
#         fi
#     fi
# fi

# # Check for GitLab token
# if [[ -z "${GITLAB_TOKEN:-}" ]]; then
#     print_warning "GITLAB_TOKEN environment variable is not set"
#     print_info "Branch pushed successfully, but cannot create merge request automatically"
#     print_info "You can create a merge request manually at:"
#     if [[ -n "$GITLAB_URL" ]] && [[ -n "$PROJECT_ID" ]]; then
#         echo "  ${GITLAB_URL}/-/merge_requests/new?merge_request[source_branch]=${BRANCH_NAME}&merge_request[target_branch]=${TARGET_BRANCH}"
#     else
#         echo "  Your GitLab instance -> New Merge Request -> Source: $BRANCH_NAME, Target: $TARGET_BRANCH"
#     fi
#     exit 0
# fi

# # Create merge request using GitLab API
# if [[ -z "$GITLAB_URL" ]] || [[ -z "$PROJECT_ID" ]]; then
#     print_warning "GitLab URL or Project ID not detected"
#     print_info "Branch pushed successfully, but cannot create merge request automatically"
#     print_info "Please create a merge request manually"
#     exit 0
# fi

# print_info "Creating merge request via GitLab API..."

# # Prepare description
# if [[ -z "$MR_DESCRIPTION" ]]; then
#     MR_DESCRIPTION="This merge request contains synchronized changes from:
# - Source directory: \`$SOURCE_DIR\`
# - Target branch: \`$TARGET_BRANCH\`

# ## Changed Files
# \`\`\`
# $(echo "$CHANGED_FILES" | head -50)
# \`\`\`
# "
# fi

# # Create merge request
# MR_RESPONSE=$(curl -s -w "\n%{http_code}" --request POST \
#     --header "PRIVATE-TOKEN: $GITLAB_TOKEN" \
#     --header "Content-Type: application/json" \
#     --data "{
#         \"source_branch\": \"$BRANCH_NAME\",
#         \"target_branch\": \"$TARGET_BRANCH\",
#         \"title\": \"$MR_TITLE\",
#         \"description\": $(echo "$MR_DESCRIPTION" | jq -Rs .)
#     }" \
#     "${GITLAB_URL}/api/v4/projects/${PROJECT_ID}/merge_requests")

# HTTP_CODE=$(echo "$MR_RESPONSE" | tail -1)
# MR_BODY=$(echo "$MR_RESPONSE" | sed '$d')

# if [[ "$HTTP_CODE" == "201" ]]; then
#     MR_URL=$(echo "$MR_BODY" | grep -o '"web_url":"[^"]*"' | cut -d'"' -f4 || echo "")
#     MR_IID=$(echo "$MR_BODY" | grep -o '"iid":[0-9]*' | head -1 | cut -d: -f2 || echo "")
#     print_success "Merge request created successfully!"
#     if [[ -n "$MR_URL" ]]; then
#         print_info "Merge request URL: $MR_URL"
#     elif [[ -n "$MR_IID" ]]; then
#         print_info "Merge request IID: $MR_IID"
#         print_info "View at: ${GITLAB_URL}/-/merge_requests/${MR_IID}"
#     fi
# else
#     print_error "Failed to create merge request. HTTP code: $HTTP_CODE"
#     print_error "Response: $MR_BODY"
#     print_info "Branch pushed successfully. Please create merge request manually"
#     exit 1
# fi

print_success "Sync completed successfully!"

