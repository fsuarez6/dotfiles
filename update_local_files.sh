#!/bin/sh
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Geany configuration
cp -r geany $HOME/.config
wget http://download.geany.org/contrib/tags/std.latex.tags
mv std.latex.tags $HOME/.config/geany/tags
# bashrc
rm -f $HOME/.bash_aliases
ln -s $DIR/.bash_aliases $HOME/.bash_aliases
echo "Done!"
