#!/bin/bash -e

PANDA_DIR=../..

mkdir /tmp/misra || true

# generate coverage matrix
#python tests/misra/cppcheck/addons/misra.py -generate-table > tests/misra/coverage_table

printf "\nPANDA CODE\n"
cppcheck -DPANDA -UPEDAL -DCAN3 -DUID_BASE -DEON \
         --suppressions-list=suppressions.txt \
         --dump --enable=all --inline-suppr --force \
         $PANDA_DIR/board/main.c 2>/tmp/misra/cppcheck_output.txt

python /usr/share/cppcheck/addons/misra.py $PANDA_DIR/board/main.c.dump 2> /tmp/misra/misra_output.txt || true

# strip (information) lines
cppcheck_output=$( cat /tmp/misra/cppcheck_output.txt | grep -v ": information: " ) || true
misra_output=$( cat /tmp/misra/misra_output.txt | grep -v ": information: " ) || true


printf "\nPEDAL CODE\n"
cppcheck -UPANDA -DPEDAL -UCAN3 -USTM32F4 -UIBST -UCTRLS \
         --suppressions-list=suppressions.txt \
         -I $PANDA_DIR/board/ --dump --enable=all --inline-suppr --force \
         $PANDA_DIR/board/pedal/main.c 2>/tmp/misra/cppcheck_pedal_output.txt

python /usr/share/cppcheck/addons/misra.py $PANDA_DIR/board/pedal/main.c.dump 2> /tmp/misra/misra_pedal_output.txt || true

# strip (information) lines
cppcheck_pedal_output=$( cat /tmp/misra/cppcheck_pedal_output.txt | grep -v ": information: " ) || true
misra_pedal_output=$( cat /tmp/misra/misra_pedal_output.txt | grep -v ": information: " ) || true

printf "\nBETTER PEDAL CODE\n"
cppcheck -UPANDA -DPEDAL -UCAN3 \
         --suppressions-list=suppressions.txt \
         -I $PANDA_DIR/board/ --dump --enable=all --inline-suppr --force \
         $PANDA_DIR/board/better_pedal/main.c 2>/tmp/misra/cppcheck_better_pedal_output.txt

python /usr/share/cppcheck/addons/misra.py $PANDA_DIR/board/better_pedal/main.c.dump 2> /tmp/misra/misra_better_pedal_output.txt || true

# strip (information) lines
cppcheck_better_pedal_output=$( cat /tmp/misra/cppcheck_better_pedal_output.txt | grep -v ": information: " ) || true
misra_better_pedal_output=$( cat /tmp/misra/misra_better_pedal_output.txt | grep -v ": information: " ) || true

printf "\nIBST CODE\n"
cppcheck -UPANDA -DIBST -UCAN3 -UPEDAL\
         --suppressions-list=suppressions.txt \
         -I $PANDA_DIR/board/ --dump --enable=all --inline-suppr --force \
         $PANDA_DIR/board/ibst/main.c 2>/tmp/misra/cppcheck_ibst_output.txt

python /usr/share/cppcheck/addons/misra.py $PANDA_DIR/board/ibst/main.c.dump 2> /tmp/misra/misra_ibst_output.txt || true

# strip (information) lines
cppcheck_ibst_output=$( cat /tmp/misra/cppcheck_ibst_output.txt | grep -v ": information: " ) || true
misra_ibst_output=$( cat /tmp/misra/misra_ibst_output.txt | grep -v ": information: " ) || true

printf "\nCTRLS CODE\n"
cppcheck -UPANDA -DCTRLS -UCAN3 -UPEDAL -USTM32F4 -UIBST\
         --suppressions-list=suppressions.txt \
         -I $PANDA_DIR/board/ --dump --enable=all --inline-suppr --force \
         $PANDA_DIR/board/ctrls/main.c 2>/tmp/misra/cppcheck_ctrls_output.txt

python /usr/share/cppcheck/addons/misra.py $PANDA_DIR/board/ctrls/main.c.dump 2> /tmp/misra/misra_ctrls_output.txt || true

# strip (information) lines
cppcheck_ctrls_output=$( cat /tmp/misra/cppcheck_ctrls_output.txt | grep -v ": information: " ) || true
misra_ctrls_output=$( cat /tmp/misra/misra_ctrls_output.txt | grep -v ": information: " ) || true

printf "\nKOMBI CODE\n"
cppcheck -UPANDA -DKOMBI -UCAN3 -UPEDAL -USTM32F4 -UIBST -UCTRKS\
         --suppressions-list=suppressions.txt \
         -I $PANDA_DIR/board/ --dump --enable=all --inline-suppr --force \
         $PANDA_DIR/board/kombi/main.c 2>/tmp/misra/cppcheck_kombi_output.txt

python /usr/share/cppcheck/addons/misra.py $PANDA_DIR/board/kombi/main.c.dump 2> /tmp/misra/misra_kombi_output.txt || true

# strip (information) lines
cppcheck_kombi_output=$( cat /tmp/misra/cppcheck_kombi_output.txt | grep -v ": information: " ) || true
misra_kombi_output=$( cat /tmp/misra/misra_kombi_output.txt | grep -v ": information: " ) || true

if [[ -n "$misra_output" ]] || [[ -n "$cppcheck_output" ]]
then
  echo "Failed! found Misra violations in panda code:"
  echo "$misra_output"
  echo "$cppcheck_output"
  exit 1
fi

if [[ -n "$misra_pedal_output" ]] || [[ -n "$cppcheck_pedal_output" ]]
then
  echo "Failed! found Misra violations in pedal code:"
  echo "$misra_pedal_output"
  echo "$cppcheck_pedal_output"
  exit 1
fi

if [[ -n "$misra_better_pedal_output" ]] || [[ -n "$cppcheck_better_pedal_output" ]]
then
  echo "Failed! found Misra violations in better_pedal code:"
  echo "$misra_better_pedal_output"
  echo "$cppcheck_better_pedal_output"
  exit 1
fi

if [[ -n "$misra_ibst_output" ]] || [[ -n "$cppcheck_ibst_output" ]]
then
  echo "Failed! found Misra violations in ibst code:"
  echo "$misra_ibst_output"
  echo "$cppcheck_ibst_output"
  exit 1
fi

if [[ -n "$misra_ctrls_output" ]] || [[ -n "$cppcheck_ctrls_output" ]]
then
  echo "Failed! found Misra violations in ctrls code:"
  echo "$misra_ctrls_output"
  echo "$cppcheck_ctrls_output"
  exit 1
fi

if [[ -n "$misra_kombi_output" ]] || [[ -n "$cppcheck_kombi_output" ]]
then
  echo "Failed! found Misra violations in kombi code:"
  echo "$misra_kombi_output"
  echo "$cppcheck_kombi_output"
  exit 1
fi
echo "Success"
