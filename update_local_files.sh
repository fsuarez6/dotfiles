#!/bin/sh
BASEDIR=$(dirname "$0")

# .bash_aliases
DST=$HOME/.bash_aliases
if [ -f $DST ] || [ -L $DST ]; then
  rm $DST
fi
ln -s `realpath $BASEDIR/.bash_aliases` $DST

# Atom
CONFIG_FILES=$(find $BASEDIR/.atom -type f -name *.cson)
for FILEPATH in $CONFIG_FILES; do
  FILENAME=$(basename "$FILEPATH")
  DST=$HOME/.atom/$FILENAME
  if [ -f $DST ] || [ -L $DST ]; then
    rm $DST
  fi
  ln -s `realpath $FILEPATH` $DST
done

echo "Done!"
