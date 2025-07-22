#! /bin/bash

echo "start setup klippy"

if [ "$#" -lt 2 ]; then
     echo  "usage: $0  insrc  outdest"
     exit 1
fi
insrc=$1
outdest=$2 


if [[ "${insrc}" != */ ]]; then
    echo "for consistency path"
    insrc="${insrc}/"
fi

if [[ "${outdest}" != */ ]]; then
    echo "for consistency path"
    outdest="${outdest}/"
fi

echo "p1: ${insrc}"
echo "p2: ${outdest}"

progdir="klippy/"
kinsubdir="kinematics/"
csubdir="chelper/"
esubdir="extras/"
edsubdir="${esubdir}display/"
progverf=".version"

mdir_kin="${outdest}${progdir}${kinsubdir}"
mdir_csub="${outdest}${progdir}${csubdir}"
mdir_edsub="${outdest}${progdir}${edsubdir}"

inmdir_kin="${insrc}${progdir}${kinsubdir}"
inmdir_csub="${insrc}${progdir}${csubdir}"
inmdir_edsub="${insrc}${progdir}${edsubdir}"


fprog="${progdir}*.pyc"
fkin="${progdir}${kinsubdir}*.pyc"
fcf="${progdir}${csubdir}*.pyc"
fef="${progdir}${esubdir}*.pyc"
fedf="${progdir}${edsubdir}*.pyc"
fcfso="${progdir}${csubdir}*.so"


if [ ! -d "$mdir_kin" ]; then
    echo "dir is not exist,creat: $mdir_kin"
    mkdir -p "$mdir_kin"  
else
    echo "dir is exist: $mdir_kin"
fi

if [ ! -d "$mdir_csub" ]; then
    echo "dir is not exist,creat: $mdir_csub"
    mkdir -p "$mdir_csub"  
else
    echo "dir is exist: $mdir_csub"
fi

if [ ! -d "$mdir_edsub" ]; then
    echo "dir is not exist,creat: $mdir_edsub"
    mkdir -p "$mdir_edsub"  
else
    echo "dir is exist: $mdir_edsub"
fi


if [ ! -d "$inmdir_kin" ]; then
    echo "failure,exit to please check: $inmdir_kin"
    exit 1
else
    echo "successful: $inmdir_kin"
fi

if [ ! -d "$inmdir_csub" ]; then
    echo "failure,exit to please check: $inmdir_csub"
    exit 1
else
    echo "successful: $inmdir_csub"
fi

if [ ! -d "$inmdir_edsub" ]; then
    echo "failure,exit to please check: $inmdir_edsub"
    exit 1
else
    echo "successful: $inmdir_edsub"
fi


if [ ! -s "${insrc}${progdir}${progverf}" ]; then
    echo "failure,exit to please check: ${insrc}${progdir}${progverf}"
    exit 1
else
    echo "successful: ${insrc}${progdir}${progverf}"
fi


echo  "copying from ${insrc}${fprog} to ${outdest}${progdir}"
cp   ${insrc}${fprog}  ${outdest}${progdir}
cp   ${insrc}${progdir}${progverf}  ${outdest}${progdir}

echo  "copying from ${insrc}${fkin} to ${outdest}${progdir}${kinsubdir}"
cp  ${insrc}${fkin}    ${outdest}${progdir}${kinsubdir}

echo  "copying from ${insrc}${fef} to ${outdest}${progdir}${esubdir}"
cp  ${insrc}${fef}     ${outdest}${progdir}${esubdir}

echo  "copying from ${insrc}${fedf} to ${outdest}${progdir}${edsubdir}"
cp  ${insrc}${fedf}    ${outdest}${progdir}${edsubdir}

echo  "copying from ${insrc}${fcf} to ${outdest}${progdir}${csubdir}"
cp  ${insrc}${fcf}     ${outdest}${progdir}${csubdir}

echo  "copying from ${insrc}${fcfso} to ${outdest}${progdir}${csubdir}"
cp  ${insrc}${fcfso}   ${outdest}${progdir}${csubdir}

echo "end setup klippy"

