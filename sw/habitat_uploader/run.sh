#!/bin/sh
KEY="" # <-- insert key

if [ -z "$KEY" ]
then
	echo "Key empty. Edit run.sh to insert a key."
	exit 1
fi

python3 habitat_uploader.py \
	--app_id balloons@ttn \
	--access_key "$KEY" "$@"
