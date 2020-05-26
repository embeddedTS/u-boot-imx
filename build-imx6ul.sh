#!/bin/bash -x

export DATE=$(date +"%b-%d-%Y")

FAIL=0

rm -rf out
mkdir out > /dev/null 2>&1
make clean && make distclean && make mrproper

case "$1" in

ts4100)
	make ts4100-512m_defconfig
	make -j9 u-boot.imx
	if [ $? -eq 0 ]; then
		mv u-boot.imx out/u-boot-ts4100-512m.imx
	else
		let FAIL=FAIL+1
	fi

	make ts4100-1g_defconfig
	make -j9 u-boot.imx
	if [ $? -eq 0 ]; then
		mv u-boot.imx out/u-boot-ts4100-1g.imx
	else
		let FAIL=FAIL+1
	fi
	;;

ts7180)
	make ts7180-512m_defconfig
	make -j9 u-boot.imx
	if [ $? -eq 0 ]; then
		mv u-boot.imx out/u-boot-ts7180-512m.imx
	else
		let FAIL=FAIL+1
	fi

	make ts7180-1g_defconfig
	make -j9 u-boot.imx
	if [ $? -eq 0 ]; then
		mv u-boot.imx out/u-boot-ts7180-1g.imx
	else
		let FAIL=FAIL+1
	fi
	;;

ts7100)
	make ts7100-512m_defconfig
	make -j9 u-boot.imx
	if [ $? -eq 0 ]; then
		mv u-boot.imx out/u-boot-ts7100-512m.imx
	else
		let FAIL=FAIL+1
	fi

#	make ts7180-1g_defconfig
#	make -j9 u-boot.imx
#	if [ $? -eq 0 ]; then
#		mv u-boot.imx out/u-boot-ts7180-1g.imx
#	else
#		let FAIL=FAIL+1
#	fi
	;;

ts7553v2)
	make ts7553v2_defconfig
	make -j9 u-boot.imx
	if [ $? -eq 0 ]; then
		mv u-boot.imx out/u-boot-ts7553v2.imx
	else
		let FAIL=FAIL+1
	fi
	;;

tsmflow)
	make tsmflow_defconfig
	make -j9 u-boot.imx
	if [ $? -eq 0 ]; then
		mv u-boot.imx out/u-boot-tsmflow-512m.imx
	else
		let FAIL=FAIL+1
	fi
	;;

*)
	echo "Usage: ./build-imx6ul.sh <model>"
	exit 1
esac

if [ $FAIL != 0 ]; then
	echo "$FAIL BUILDS FAILED.  DO NOT RELEASE"
	exit 1
fi

exit 0
