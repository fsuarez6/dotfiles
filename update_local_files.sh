#!/bin/sh

# Geany configuration
cp -r geany $HOME/.config
wget http://download.geany.org/contrib/tags/std.latex.tags
mv std.latex.tags $HOME/.config/geany/tags
# Terminator configuration
cp -r terminator $HOME/.config
# bashrc
cp .bash_aliases $HOME
echo "Done!"
