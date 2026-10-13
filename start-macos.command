#!/bin/sh
cd -- "$(dirname -- "$0")" || exit 1

if ! command -v node >/dev/null 2>&1; then
  printf '%s\n' 'Node.js 20 or newer is required. Install Node.js, then reopen this launcher.'
  printf '%s' 'Press Enter to close... '
  read -r reply
  exit 1
fi

printf '%s\n' 'Open http://127.0.0.1:4173 in your browser.' 'Keep this window open. Press Ctrl+C to stop.'
node server.mjs
result=$?
if [ "$result" -ne 0 ]; then
  printf '%s' 'The server stopped with an error. Press Enter to close... '
  read -r reply
fi
exit "$result"
