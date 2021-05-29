#!/bin/bash

pre-commit install

git add .
git commit -m "Automated commit via 'start_new_repo.sh'"
git push -u origin master
