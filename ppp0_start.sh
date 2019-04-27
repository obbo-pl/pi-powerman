#!/bin/sh

/sbin/ifup --force ppp0 || exit 1

exit 0

