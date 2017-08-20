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
    PSCol="$Yel"
elif [ $HOSTNAME == 'zbook' ]; then
    PSCol="$Pur"
elif [ $HOSTNAME == 'hp' ]; then                  # Research fellow room
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
    echo "Converting BLACK ==> TRANSPARENT ($ALPHA)"
    convert $PNG -transparent black $ALPHA
  done
}

# Custom Aliases
alias up='cd ..'
alias up2='up && up'
alias up3='up2 && up'
alias up4='up3 && up'
alias killgazebo="killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient"

# LaTeX paper
PAPER_PATH=~/git/cri/papers/manipulation
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
source ~/.rvm/scripts/rvm

# Denso
alias denso_home="roslaunch denso_manipulation move_home_position.launch"
alias denso_fake_left="roslaunch denso_control denso_fake_controller.launch robot_name:=left"
alias denso_fake_right="roslaunch denso_control denso_fake_controller.launch robot_name:=right"
alias denso_left_rrt="roslaunch denso_control denso_rrt_controller.launch robot_name:=left denso_address:=192.168.0.11 netft_address:=192.168.0.12"
alias denso_right_rrt="roslaunch denso_control denso_rrt_controller.launch robot_name:=right denso_address:=192.168.0.21 netft_address:=192.168.0.22"
alias denso_left_rrt_read="roslaunch denso_control denso_rrt_controller.launch robot_name:=left motor_on:=False denso_address:=192.168.0.11 netft_address:=192.168.0.12"
alias denso_right_rrt_read="roslaunch denso_control denso_rrt_controller.launch robot_name:=right motor_on:=False denso_address:=192.168.0.21 netft_address:=192.168.0.22"
alias stopmongo="sudo service mongodb stop"
alias rosplan_rqt="rqt --standalone rosplan_rqt"
alias ueyerestart="sudo /etc/init.d/ueyeethdrc force-stop && sudo /etc/init.d/ueyeethdrc start"

ROS_WS_PATH=~/catkin_ws

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
export OSG_NOTIFY_LEVEL=WARN

source $ROS_WS_PATH/devel/setup.bash

# Gitbook
NPM_PACKAGES="~/.npm_packages"
PATH="$NPM_PACKAGES/bin:$PATH"
# Unset manpath so we can inherit from /etc/manpath via the `manpath` command
unset MANPATH # delete if you already modified MANPATH elsewhere in your config
export MANPATH="$NPM_PACKAGES/share/man:$(manpath)"

# Gurobi
export GUROBI_HOME="/opt/gurobi751/linux64"
export PATH="${PATH}:${GUROBI_HOME}/bin"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"
export GRB_LICENSE_FILE="${GUROBI_HOME}/gurobi.lic"
# SCIP Optimization Suite
export SCIPOPTDIR=/opt/scip-4.0.0
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${SCIPOPTDIR}/lib"
export PATH="${PATH}:${UNIX_SCIP_DIR}/bin"
