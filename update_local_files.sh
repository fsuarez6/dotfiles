#!/bin/sh

# Geany configuration
wget http://download.geany.org/contrib/tags/std.latex.tags
mv std.latex.tags $HOME/.config/geany/tags
cp -r geany $HOME/.config
# bashrc
cp .bash_aliases $HOME
echo "Done!"
