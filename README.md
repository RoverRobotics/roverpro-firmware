firmware
========

Firmware

To tidy up code, use clang-format. I installed this with
```
sudo apt install clang-format-6.0
sudo ln -s `which clang-format-6.0` /usr/local/bin/clang-format
```
Before committing, please run
```
git diff --name-only --cached --relative -- '*.h' '*.c' | xargs clang-format -style=file -i
```
This will tidy up all locally modified files. You can make this a precommit hook if you like.
