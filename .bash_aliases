# Custom colors for ssh
RCol='\[\e[0m\]'    # Text Reset
Blk='\[\e[0;30m\]'  # Black
Red='\[\e[0;31m\]'  # Red
Gre='\[\e[0;32m\]'  # Green
Yel='\[\e[0;33m\]'  # Yellow
Blu='\[\e[0;34m\]'  # Blue
Pur='\[\e[0;35m\]'  # Purple
Cya='\[\e[0;36m\]'  # Cyan
Whi='\[\e[0;37m\]'  # White

PSCol="$Whi"
if [ $HOSTNAME == 'thinkstation' ]; then
    PSCol="$Blu"
elif [ $HOSTNAME == 'dellstation' ]; then
    PSCol="$Cya"
elif [ $HOSTNAME == 'rogsus' ]; then
    PSCol="$Gre"
elif [ $HOSTNAME == 'zbook' ]; then
    PSCol="$Pur"
elif [ $HOSTNAME == 'ideapad' ]; then             # Personal netbook
    PSCol="$Yel"
elif [ $HOSTTYPE == 'arm' ]; then                 # Raspberry pi
    PSCol="$Gre"
elif [[ $MACHTYPE =~ arm-apple-darwin ]]; then    # iOS
    PSCol="$Red"
elif [ $MACHTYPE == 'i486-pc-linux-gnu' ]; then   # For Netbook
    PSCol="$Whi"
fi

PS1="${PSCol}\u@\h${RCol}:\w\$ "

# pdf2png
function pdf2png() {
  CURRENTDIR=`pwd`;
  FILES=$(find $CURRENTDIR -maxdepth 1 -type f -name '*.pdf')
  for FILEPATH in $FILES
  do
    PDF=$(basename "$FILEPATH")
    FILENAME="${PDF%.*}"
    PNG=$FILENAME.png
    echo "Converting $PDF ==> $PNG"
    convert -density 200 -trim $PDF -quality 100 -sharpen 0x1.0 $PNG
  done
}

# add_transparency
function alpha2png() {
  CURRENTDIR=`pwd`;
  FILES=$(find $CURRENTDIR -maxdepth 1 -type f -name '*.png')
  for FILEPATH in $FILES
  do
    PNG=$(basename "$FILEPATH")
    FILENAME="${PNG%.*}"
    ALPHA=$FILENAME-alpha.png
    echo "Converting WHITE ==> TRANSPARENT ($ALPHA)"
    convert $PNG -transparent white $ALPHA
  done
}

# Custom Aliases
alias up='cd ..'
alias up2='up && up'
alias up3='up2 && up'
alias up4='up3 && up'
alias killgazebo="killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient"

# LaTeX paper
PAPER_PATH=~/git/tase_2018
function paper_labels() {
  find $PAPER_PATH \( -path $PAPER_PATH/doc -prune -o -name '*.tex' \) | sort | xargs grep --color -oPnH '(?<=\\label\{)(.*?)(?=\})'; }
function paper_cites() {
  find $PAPER_PATH \( -path $PAPER_PATH/doc -prune -o -name '*.tex' \) | sort | xargs grep --color -oPnH '(?<=\\cite\{)(.*?)(?=\})'; }
function paper_log() {
  LOG_FILE=$PAPER_PATH/main.log
  grep -rn 'LaTeX Warning:' $LOG_FILE;
  grep -rn '! ' $LOG_FILE; }
alias paper_build='cd $PAPER_PATH && scons -Q && cd -'
alias paper_view='cd $PAPER_PATH && evince main.pdf > /dev/null & cd - > /dev/null'

# Jekyll
if [ -f ~/.rvm/scripts/rvm ]; then
  source ~/.rvm/scripts/rvm
fi

# Ensenso
alias ueyerestart="sudo /etc/init.d/ueyeethdrc force-stop && sudo /etc/init.d/ueyeethdrc start"

# Gitbook
NPM_PACKAGES="~/.npm_packages"
PATH="$NPM_PACKAGES/bin:$PATH"
# Unset manpath so we can inherit from /etc/manpath via the `manpath` command
unset MANPATH # delete if you already modified MANPATH elsewhere in your config
export MANPATH="$NPM_PACKAGES/share/man:$(manpath)"

# # Gurobi
# export GUROBI_HOME="/opt/gurobi751/linux64"
# export PATH="${PATH}:${GUROBI_HOME}/bin"
# export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"
# export GRB_LICENSE_FILE="${GUROBI_HOME}/gurobi.lic"
# # SCIP Optimization Suite
# export SCIPOPTDIR=/opt/scip-4.0.0
# export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${SCIPOPTDIR}/lib"
# export PATH="${PATH}:${UNIX_SCIP_DIR}/bin"

# ROS_WS_PATH=~/ws_lenny
ROS_WS_PATH=~/catkin_ws
# ROS_WS_PATH=~/ws_osrobotics

# ROS Staff
alias cd_catkin='cd $ROS_WS_PATH/src'
EDITOR=atom
function rosbuild()
{
  cd $ROS_WS_PATH   ;
  catkin_make "$@"  ;
  cd - > /dev/null  ;
}
alias rqt_reset="rm ~/.config/ros.org/rqt_gui.ini"
function rosdep_check()
{
  cd $ROS_WS_PATH/src ;
  rosdep check --from-paths . --ignore-src --rosdistro $ROS_DISTRO ;
  cd - > /dev/null ;
}
export OSG_NOTIFY_LEVEL=WARN

# retro-contest
alias ipy="python -c 'import IPython; IPython.terminal.ipapp.launch_new_instance()'"
export DOCKER_REGISTRY="retrocontestapqlnovcmrexyfsw.azurecr.io"
export WORKON_HOME=~/.virtualenvs
source ~/.local/bin/virtualenvwrapper.sh
function retro-build()
{
  cd $DOCKER_PATH   ;
  docker build -f $DOCKER_FILE -t $DOCKER_REGISTRY/$DOCKER_TAG .  ;
  cd - > /dev/null  ;
}

function retro-test()
{
  retro-build   ;
  retro-contest run --agent $DOCKER_REGISTRY/$DOCKER_TAG                      \
    --results-dir results --no-nv --use-host-data --timestep-limit $TIMESTEPS \
    SonicTheHedgehog-Genesis GreenHillZone.Act1 ;
}

# workon retro
# Source ROS
if [ -f $ROS_WS_PATH/devel/setup.bash ]; then
  source $ROS_WS_PATH/devel/setup.bash
elif [ -f /opt/ros/kinetic/setup.bash ]; then
  source /opt/ros/kinetic/setup.bash
fi
