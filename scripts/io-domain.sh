#!/bin/sh
# SPDX-License-Identifier: (GPL-2.0+ OR MIT)
# Copyright (c) 2021 Rockchip Electronics Co., Ltd.

PMUIO2=0
VCCIO1=0
VCCIO3=0
VCCIO4=0
VCCIO5=0
VCCIO6=0
VCCIO7=0

DTS_PMUIO2=0
DTS_VCCIO1=0
DTS_VCCIO3=0
DTS_VCCIO4=0
DTS_VCCIO5=0
DTS_VCCIO6=0
DTS_VCCIO7=0

DTS_NAME=$1.dts.tmp

CheckBckfileRet=0
checklistRst=0
GetVoltageFromDtsVal=0

ShowChecklist()
{
	PMUIO2=$(whiptail --title "IO Domain Checklist" --menu --nocancel\
	"Get the corresponding value from the hardware schematic diagram" 15 60 2 \
	"1800000" "PMUIO2 Supply Power Voltage(uV)" \
	"3300000" "PMUIO2 Supply Power Voltage(uV)" 3>&1 1>&2 2>&3)
	exitstatus=$?
	if [ $exitstatus != 0 ]; then
		echo "You chose Cancel."
		checklistRst=1
	fi

	VCCIO1=$(whiptail --title "IO Domain Checklist" --menu --nocancel\
	"Get the corresponding value from the hardware schematic diagram" 15 60 2 \
	"1800000" "VCCIO1 Supply Power Voltage(uV)" \
	"3300000" "VCCIO1 Supply Power Voltage(uV)" 3>&1 1>&2 2>&3)
	exitstatus=$?
	if [ $exitstatus != 0 ]; then
		echo "You chose Cancel."
		checklistRst=1
	fi

	VCCIO3=$(whiptail --title "IO Domain Checklist" --menu --nocancel\
	"Get the corresponding value from the hardware schematic diagram" 15 60 2 \
	"1800000" "VCCIO3 Supply Power Voltage(uV)" \
	"3300000" "VCCIO3 Supply Power Voltage(uV)" 3>&1 1>&2 2>&3)
	exitstatus=$?
	if [ $exitstatus != 0 ]; then
		echo "You chose Cancel."
		checklistRst=1
	fi

	VCCIO4=$(whiptail --title "IO Domain Checklist" --menu --nocancel\
	"Get the corresponding value from the hardware schematic diagram" 15 60 2 \
	"1800000" "VCCIO4 Supply Power Voltage(uV)" \
	"3300000" "VCCIO4 Supply Power Voltage(uV)" 3>&1 1>&2 2>&3)
	exitstatus=$?
	if [ $exitstatus != 0 ]; then
		echo "You chose Cancel."
		checklistRst=1
	fi

	VCCIO5=$(whiptail --title "IO Domain Checklist" --menu --nocancel\
	"Get the corresponding value from the hardware schematic diagram" 15 60 2 \
	"1800000" "VCCIO5 Supply Power Voltage(uV)" \
	"3300000" "VCCIO5 Supply Power Voltage(uV)" 3>&1 1>&2 2>&3)
	exitstatus=$?
	if [ $exitstatus != 0 ]; then
		echo "You chose Cancel."
		checklistRst=1
	fi

	VCCIO6=$(whiptail --title "IO Domain Checklist" --menu --nocancel\
	"Get the corresponding value from the hardware schematic diagram" 15 60 2 \
	"1800000" "VCCIO6 Supply Power Voltage(uV)" \
	"3300000" "VCCIO6 Supply Power Voltage(uV)" 3>&1 1>&2 2>&3)
	exitstatus=$?
	if [ $exitstatus != 0 ]; then
		echo "You chose Cancel."
		checklistRst=1
	fi

	VCCIO7=$(whiptail --title "IO Domain Checklist" --menu --nocancel\
	"Get the corresponding value from the hardware schematic diagram" 15 60 2 \
	"1800000" "VCCIO7 Supply Power Voltage(uV)" \
	"3300000" "VCCIO7 Supply Power Voltage(uV)" 3>&1 1>&2 2>&3)
	exitstatus=$?
	if [ $exitstatus != 0 ]; then
		echo "You chose Cancel."
		checklistRst=1
	fi
}

DtsIoDomainVoltageVal=0
DtsIoDomainVoltage()
{
	DtsIoDomainVoltageVal=0
	flags=0
	if [ -f $DTS_NAME ];then
		echo "found $DTS_NAME"
	fi

	supply=$(cat $DTS_NAME \
		| grep $1 \
		| cut -d "&" -f 2 \
		| cut -d ">" -f 1 \
		| tail -n1)

	ldo_str=$(cat $DTS_NAME  \
		| awk 'BEGIN {RS="\n\n+";ORS="\n\n"}/regulator-name/{print $0}' \
		| awk BEGIN{RS=EOF}'{gsub(/\n/," ");print $0}' \
		| grep $supply \
		| awk '{print $2}' \
		| awk -F 'SWITCH_REG' '{print $2}')

	if [ "$ldo_str" != "" ];then
		DtsIoDomainVoltageVal=3300000
	else
		DtsIoDomainVoltageVal=$(cat $DTS_NAME  \
			| awk 'BEGIN {RS="\n\n+";ORS="\n\n"}/regulator-name/{print $0}' \
			| awk BEGIN{RS=EOF}'{gsub(/\n/," ");print $0}' \
			|grep $supply \
			| awk -F 'regulator-max-microvolt' '{print $2}' \
			| cut -d "<" -f 2 \
			| cut -d ">" -f 1)
	fi
}

GetIoDomainVoltageFromDts()
{
	DtsIoDomainVoltage "pmuio2-supply"
	DTS_PMUIO2=$DtsIoDomainVoltageVal
	DtsIoDomainVoltage "vccio1-supply"
	DTS_VCCIO1=$DtsIoDomainVoltageVal
	DtsIoDomainVoltage "vccio3-supply"
	DTS_VCCIO3=$DtsIoDomainVoltageVal
	DtsIoDomainVoltage "vccio4-supply"
	DTS_VCCIO4=$DtsIoDomainVoltageVal
	DtsIoDomainVoltage "vccio5-supply"
	DTS_VCCIO5=$DtsIoDomainVoltageVal
	DtsIoDomainVoltage "vccio6-supply"
	DTS_VCCIO6=$DtsIoDomainVoltageVal
	DtsIoDomainVoltage "vccio7-supply"
	DTS_VCCIO7=$DtsIoDomainVoltageVal
}

CheckVoltageWithBackupfile()
{
	CheckBckfileRet=2
	val=$(cat $DTS_NAME.domain \
			| grep PMUIO2 \
			| cut -d ":" -f 2)
	if [ "$val" != "$DTS_PMUIO2" ];then
		CheckBckfileRet=1
		echo "PMUIO2 Supply Power Voltage has changed!!! please reconfirm!!!"
	fi
	val=$(cat $DTS_NAME.domain \
			| grep VCCIO1 \
			| cut -d ":" -f 2)
	if [ "$val" != "$DTS_VCCIO1" ];then
		CheckBckfileRet=1
		echo "VCCIO1 Supply Power Voltage has changed!!! please reconfirm!!!"
	fi
	val=$(cat $DTS_NAME.domain \
			| grep VCCIO3 \
			| cut -d ":" -f 2)
	if [ "$val" != "$DTS_VCCIO3" ];then
		CheckBckfileRet=1
		echo "VCCIO3 Supply Power Voltage has changed!!! please reconfirm!!!"
	fi
	val=$(cat $DTS_NAME.domain | grep VCCIO4 | cut -d ":" -f 2)
	if [ "$val" != "$DTS_VCCIO4" ];then
		ret=1
		echo "VCCIO4 Supply Power Voltage has changed!!! please reconfirm!!!"
	fi
	val=$(cat $DTS_NAME.domain \
			| grep VCCIO5 \
			| cut -d ":" -f 2)
	if [ "$val" != "$DTS_VCCIO5" ];then
		CheckBckfileRet=1
		echo "VCCIO5 Supply Power Voltage has changed!!! please reconfirm!!!"
	fi
	val=$(cat $DTS_NAME.domain \
			| grep VCCIO6 \
			| cut -d ":" -f 2)
	if [ "$val" != "$DTS_VCCIO6" ];then
		CheckBckfileRet=1
		echo "VCCIO6 Supply Power Voltage has changed!!! please reconfirm!!!"
	fi
	val=$(cat $DTS_NAME.domain \
			| grep VCCIO7 \
			| cut -d ":" -f 2)
	if [ "$val" != "$DTS_VCCIO7" ];then
		CheckBckfileRet=1
		echo "VCCIO7 Supply Power Voltage has changed!!! please reconfirm!!!"
	fi
}

CheckVoltageWithEnter()
{
	checklistRst=0
	if [ $PMUIO2 -ne $DTS_PMUIO2 ];then
		checklistRst=1
		echo "PMUIO2 Supply Power Voltage has changed!!! please reconfirm!!!"
	fi
	if [ $VCCIO1 -ne $DTS_VCCIO1 ];then
		checklistRst=1
		echo "VCCIO1 Supply Power Voltage has changed!!! please reconfirm!!!"
	fi
	if [ $VCCIO3 -ne $DTS_VCCIO3 ];then
		checklistRst=1
		echo "VCCIO3 Supply Power Voltage has changed!!! please reconfirm!!!"
	fi
	if [ $VCCIO4 -ne $DTS_VCCIO4 ];then
		checklistRst=1
		echo "VCCIO4 Supply Power Voltage has changed!!! please reconfirm!!!"
	fi
	if [ $VCCIO5 -ne $DTS_VCCIO5 ];then
		checklistRst=1
		echo "VCCIO5 Supply Power Voltage has changed!!! please reconfirm!!!"
	fi
	if [ $VCCIO6 -ne $DTS_VCCIO6 ];then
		checklistRst=1
		echo "VCCIO6 Supply Power Voltage has changed!!! please reconfirm!!!"
	fi
	if [ $VCCIO7 -ne $DTS_VCCIO7 ];then
		checklistRst=1
		echo "VCCIO7 Supply Power Voltage has changed!!! please reconfirm!!!"
	fi
}

IsRK356x=$(cat $DTS_NAME | grep sdhci@fe310000)
if [ "$IsRK356x" = "" ];then
	echo "is not rk356x"
	exit 0
fi

GetIoDomainVoltageFromDts
if [ -f $DTS_NAME.domain ];then
	CheckVoltageWithBackupfile
fi

if [ "$CheckBckfileRet" != "2" ];then
	ShowChecklist
	CheckVoltageWithEnter
	if [ $checklistRst -eq 0 ]; then
		if [ -f $DTS_NAME.domain ];then
			rm -rf $DTS_NAME.domain
		fi

		echo "PMUIO2 Supply Power Voltage1:$PMUIO2" >> $DTS_NAME.domain
		echo "VCCIO1 Supply Power Voltage1:$VCCIO1" >> $DTS_NAME.domain
		echo "VCCIO3 Supply Power Voltage1:$VCCIO3" >> $DTS_NAME.domain
		echo "VCCIO4 Supply Power Voltage1:$VCCIO4" >> $DTS_NAME.domain
		echo "VCCIO5 Supply Power Voltage1:$VCCIO5" >> $DTS_NAME.domain
		echo "VCCIO6 Supply Power Voltage1:$VCCIO6" >> $DTS_NAME.domain
		echo "VCCIO7 Supply Power Voltage1:$VCCIO7" >> $DTS_NAME.domain
		echo 0
	else
		echo "io-domian  default as:
		    	&pmu_io_domains {
				status = "okay";
				pmuio1-supply = <&vcc3v3_pmu>;
				pmuio2-supply = <&vcc3v3_pmu>;
				vccio1-supply = <&vccio_acodec>;
				vccio3-supply = <&vccio_sd>;
				vccio4-supply = <&vcc_3v3>;
				vccio5-supply = <&vcc_3v3>;
				vccio6-supply = <&vcc_3v3>;
				vccio7-supply = <&vcc_3v3>;
			};
			TODO:
			Need to be modified according to the actual hardware
			for example rk3568-evb:
			&pmu_io_domains {
				status = "okay";
				pmuio1-supply = <&vcc3v3_pmu>;
				pmuio2-supply = <&vcc3v3_pmu>;
				vccio1-supply = <&vccio_acodec>;
				vccio3-supply = <&vccio_sd>;
				vccio4-supply = <&vcc_1v8>;
				vccio5-supply = <&vcc_3v3>;
				vccio6-supply = <&vcc_1v8>;
				vccio7-supply = <&vcc_3v3>;
			};

		io-domain docs:

		Android11 SDK:
		RKDocs/android/Rockchip_Developer_Guide_Android11_SDK_V1.1.2_CN.pdf or newer.

		RK356X Linux SDK:
		docs/RK356X/Rockchip_RK356X_Introduction_IO_Power_Domains_Configuration.pdf
		docs/Common/IO-DOMAIN/Rockchip_Developer_Guide_Linux_IO_DOMAIN_CN.pdf

		"

		exit 1
	fi
fi
