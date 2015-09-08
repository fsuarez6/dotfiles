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
elif [ $HOSTNAME == 'ideapad' ]; then
    PSCol="$Yel"
elif [ $HOSTNAME == 'pavilion' ]; then
    PSCol="$Pur"
elif [ $HOSTNAME == 'kratos' ]; then
    PSCol="$Yel"
elif [ $HOSTTYPE == 'arm' ]; then
    PSCol="$Gre"                # For pi
elif [[ $MACHTYPE =~ arm-apple-darwin ]]; then
    PSCol="$Red"                # For iOS
elif [ $MACHTYPE == 'i486-pc-linux-gnu' ]; then
    PSCol="$Whi"                # For Netbook
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

# Custom Aliases
alias up='cd ..'
alias up2='up && up'
alias up3='up2 && up'
alias up4='up3 && up'
alias cd_catkin='cd ~/catkin_ws/src'
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

# Baxter
alias baxter_shell="cd ~/catkin_ws && . baxter.sh && cd -"
alias baxter_enable="rosrun baxter_tools enable_robot.py -e"
alias baxter_disable="rosrun baxter_tools enable_robot.py -d"

# Denso
alias denso_home="roslaunch denso_manipulation move_home_position.launch"
#~ alias denso_rrt="roslaunch denso_control denso_rrt_controller.launch"
alias denso_left_rrt="roslaunch denso_control denso_rrt_controller.launch robot_name:=left denso_address:=192.168.0.11 netft_address:=192.168.0.12"
alias denso_right_rrt="roslaunch denso_control denso_rrt_controller.launch robot_name:=right denso_address:=192.168.0.21 netft_address:=192.168.0.22"
alias denso_left_rrt_read="roslaunch denso_control denso_rrt_controller.launch robot_name:=left motor_on:=False denso_address:=192.168.0.11 netft_address:=192.168.0.12"
alias denso_right_rrt_read="roslaunch denso_control denso_rrt_controller.launch robot_name:=right motor_on:=False denso_address:=192.168.0.21 netft_address:=192.168.0.22"

# ROS Staff
EDITOR=geany
alias roscat="cd ~/catkin_ws && catkin_make install && cd - > /dev/null"
source ~/catkin_ws/devel/setup.bash
