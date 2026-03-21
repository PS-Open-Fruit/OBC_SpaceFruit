#!/usr/bin/env bash

set -euo pipefail

# Ask for username
read -rp "Username: " USERNAME

# Ask for password (silent)
read -rsp "Password: " PASSWORD
echo

# Epoch time in milliseconds
A="$(($(date +%s%N)/1000000))"

curl 'https://security.eng.kmutnb.ac.th:8090/login.xml' \
  -H 'Accept: */*' \
  -H 'Accept-Language: en-TH,en;q=0.9,th-TH;q=0.8,th;q=0.7,en-GB;q=0.6,en-US;q=0.5' \
  -H 'Connection: keep-alive' \
  -H 'Content-Type: application/x-www-form-urlencoded' \
  -H 'Origin: https://security.eng.kmutnb.ac.th:8090' \
  -H 'Referer: https://security.eng.kmutnb.ac.th:8090/httpclient.html' \
  -H 'Sec-Fetch-Dest: empty' \
  -H 'Sec-Fetch-Mode: cors' \
  -H 'Sec-Fetch-Site: same-origin' \
  -H 'User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/144.0.0.0 Safari/537.36' \
  -H 'sec-ch-ua: "Not(A:Brand";v="8", "Chromium";v="144", "Google Chrome";v="144"' \
  -H 'sec-ch-ua-mobile: ?0' \
  -H 'sec-ch-ua-platform: "Windows"' \
  --data-raw "mode=191&username=${USERNAME}&password=${PASSWORD}&a=${A}&producttype=0"

# Cleanup (extra safety)
unset PASSWORD
