#!/bin/bash
# ══════════════════════════════════════════════════════════════
#  Campus_Shuttle — GitHub Deployment Script
#  BITS Pilani Autonomous Shuttle Mission Control
#
#  Usage:
#    cd /path/to/Campus_Shuttle
#    bash push_to_github.sh
#
#  This script:
#   1. Initialises git if not already a repo
#   2. Sets up remote origin if needed
#   3. Configures Git LFS for the large .pcd file (optional)
#   4. Creates .gitattributes for LFS tracking
#   5. Stages all tracked files (excluding build artifacts & .pcd)
#   6. Commits with a timestamped message
#   7. Pushes to origin/main
# ══════════════════════════════════════════════════════════════

set -e  # Exit on any error

# ── Configuration ──────────────────────────────────────────────
REPO_URL="https://github.com/manish-gupta-in/Campus_Shuttle.git"
BRANCH="main"
COMMIT_MSG="chore: update Campus_Shuttle v7.0 — $(date '+%Y-%m-%d %H:%M IST')"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${CYAN}╔═══════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║   Campus_Shuttle → GitHub Deployment Tool     ║${NC}"
echo -e "${CYAN}║   BITS Pilani Autonomous Vehicle Lab          ║${NC}"
echo -e "${CYAN}╚═══════════════════════════════════════════════╝${NC}"
echo ""

# ── Step 1: Initialise Git ─────────────────────────────────────
if [ ! -d ".git" ]; then
    echo -e "${YELLOW}[1/7] Initialising new git repository...${NC}"
    git init
    git checkout -b "$BRANCH" 2>/dev/null || true
else
    echo -e "${GREEN}[1/7] Git repository already initialised.${NC}"
fi

# ── Step 2: Set remote origin ──────────────────────────────────
echo -e "${YELLOW}[2/7] Setting remote origin...${NC}"
if git remote get-url origin &>/dev/null; then
    git remote set-url origin "$REPO_URL"
    echo -e "${GREEN}      Remote origin updated: $REPO_URL${NC}"
else
    git remote add origin "$REPO_URL"
    echo -e "${GREEN}      Remote origin added: $REPO_URL${NC}"
fi

# ── Step 3: Setup Git LFS (optional) ──────────────────────────
echo -e "${YELLOW}[3/7] Checking Git LFS availability...${NC}"
if command -v git-lfs &>/dev/null; then
    echo -e "${GREEN}      Git LFS found. Initialising for .pcd files...${NC}"
    git lfs install --local
    git lfs track "*.pcd"
    echo -e "${GREEN}      ✅ LFS tracking: *.pcd files will be uploaded via LFS${NC}"
else
    echo -e "${YELLOW}      ⚠️  Git LFS not installed. Pointcloud (.pcd) excluded via .gitignore${NC}"
    echo -e "${YELLOW}      Install: sudo apt install git-lfs && git lfs install${NC}"
fi

# ── Step 4: Write .gitattributes ──────────────────────────────
echo -e "${YELLOW}[4/7] Writing .gitattributes...${NC}"
cat > .gitattributes << 'EOF'
# Git LFS — Large Binary File Tracking
*.pcd filter=lfs diff=lfs merge=lfs -text

# Line Endings
*.py    text eol=lf
*.sh    text eol=lf
*.md    text eol=lf
*.yaml  text eol=lf
*.xml   text eol=lf
*.txt   text eol=lf
*.json  text eol=lf
*.cfg   text eol=lf

# Binary Assets
*.png binary
*.jpg binary
*.jpeg binary
*.webp binary
EOF
echo -e "${GREEN}      .gitattributes written.${NC}"

# ── Step 5: Stage all files ────────────────────────────────────
echo -e "${YELLOW}[5/7] Staging files...${NC}"
git add .gitignore .gitattributes README.md ARCHITECTURE.md WALKTHROUGH.md requirements.txt
git add shuttle_dashboard.py test_dashboard.py run_dashboard.sh
git add car.jpeg wilp_logo.png logo.webp Dashboard_Image.png
git add av_ws/src/ av_ws/college.sh
git add map/lanelet2_map.osm map/map_config.yaml map/map_projector_info.yaml
git add shuttle_sim_Waypoints/

# Add LFS tracked .pcd if LFS is configured
if command -v git-lfs &>/dev/null && git lfs env &>/dev/null; then
    git add map/pointcloud_map.pcd 2>/dev/null && \
        echo -e "${GREEN}      ✅ pointcloud_map.pcd staged via Git LFS${NC}" || \
        echo -e "${YELLOW}      ⚠️  pointcloud_map.pcd skipped (check LFS setup)${NC}"
fi

echo -e "${GREEN}      Files staged successfully.${NC}"
git status --short

# ── Step 6: Commit ─────────────────────────────────────────────
echo ""
echo -e "${YELLOW}[6/7] Committing changes...${NC}"
git commit -m "$COMMIT_MSG" || echo -e "${YELLOW}      Nothing new to commit.${NC}"

# ── Step 7: Push ───────────────────────────────────────────────
echo ""
echo -e "${YELLOW}[7/7] Pushing to ${REPO_URL} (branch: ${BRANCH})...${NC}"
echo -e "${CYAN}      You may be prompted for GitHub credentials.${NC}"
echo -e "${CYAN}      Use a Personal Access Token (PAT) as password.${NC}"
echo ""
git push -u origin "$BRANCH"

echo ""
echo -e "${GREEN}╔═══════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  ✅  DEPLOYMENT COMPLETE!                     ║${NC}"
echo -e "${GREEN}║  View: https://github.com/manish-gupta-in/   ║${NC}"
echo -e "${GREEN}║        Campus_Shuttle                         ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════════════╝${NC}"
