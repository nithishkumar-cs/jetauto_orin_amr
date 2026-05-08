#!/usr/bin/env bash
set -euo pipefail

git log --date=short --pretty=format:'%ad | %h | %s' -- docs src README.md PROJECT.md

