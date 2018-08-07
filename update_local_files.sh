#!/bin/sh
BASEDIR=$(dirname "$0")

# .bash_aliases
DST=$HOME/.bash_aliases
if [ -f $DST ] || [ -L $DST ]; then
  rm $DST
fi
ln -s `realpath $BASEDIR/.bash_aliases` $DST
echo "Added symlink: $DST"

# Atom
CONFIG_FILES=$(find $BASEDIR/atom -maxdepth 1 -type f -name *.cson -o -name *.less)
for FILEPATH in $CONFIG_FILES; do
  FILENAME=$(basename "$FILEPATH")
  DST=$HOME/.atom/$FILENAME
  if [ -f $DST ] || [ -L $DST ]; then
    rm $DST
  fi
  ln -s `realpath $FILEPATH` $DST
  echo "Added symlink: $DST"
done

echo "Done!"
