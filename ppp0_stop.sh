#!/bin/sh

/sbin/ifdown --force ppp0 || exit 1

exit 0
