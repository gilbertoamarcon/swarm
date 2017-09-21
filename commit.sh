#!/bin/bash
## Commit

# I use this script to rapidly commit code to the master branch of my git repositories.
# The commit name is passed as a command line parameter.
#

git add --all
git commit -m "$1"
git push -u origin master

