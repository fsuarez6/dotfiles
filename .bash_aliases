# Custom colors for ssh
RCol='\[\e[0m\]'    # Text Reset
Red='\[\e[0;31m\]'  # Red
Gre='\[\e[0;32m\]'  # Green
Yel='\[\e[0;33m\]'  # Yellow
Blu='\[\e[0;34m\]'  # Blue
Pur='\[\e[0;35m\]'  # Purple
Cya='\[\e[0;36m\]'  # Cyan
Whi='\[\e[0;37m\]'  # White

PSCol="$Whi"
if [ $HOSTNAME == 'kronos' ]; then
    PSCol="$Blu"
elif [ $HOSTNAME == 'sonic' ]; then
    PSCol="$Cya"
elif [ $HOSTNAME == 'rosbox' ]; then
    PSCol="$Pur"
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

# LaTeX Thesis
THESIS_PATH=~/git/phd-thesis
function phd_labels() {
  find $THESIS_PATH \( -path $THESIS_PATH/doc -prune -o -name '*.tex' \) | sort | xargs grep --color -oPnH '(?<=\\label\{)(.*?)(?=\})'; }
function phd_cites() {
  find $THESIS_PATH \( -path $THESIS_PATH/doc -prune -o -name '*.tex' \) | sort | xargs grep --color -oPnH '(?<=\\cite\{)(.*?)(?=\})'; }
function phd_log() {
  LOG_FILE=$THESIS_PATH/thesis.log
  grep -rn 'LaTeX Warning:' $LOG_FILE;
  grep -rn '! ' $LOG_FILE; }
alias phd_build='cd $THESIS_PATH && scons -Q && cd -'

# LaTeX paper
PAPER_PATH=~/git/kinematics-journal
function paper_labels() {
  find $PAPER_PATH \( -path $PAPER_PATH/doc -prune -o -name '*.tex' \) | sort | xargs grep --color -oPnH '(?<=\\label\{)(.*?)(?=\})'; }
function paper_cites() {
  find $PAPER_PATH \( -path $PAPER_PATH/doc -prune -o -name '*.tex' \) | sort | xargs grep --color -oPnH '(?<=\\cite\{)(.*?)(?=\})'; }
function paper_log() {
  LOG_FILE=$PAPER_PATH/paper.log
  grep -rn 'LaTeX Warning:' $LOG_FILE;
  grep -rn '! ' $LOG_FILE; }
alias paper_build='cd $PAPER_PATH && scons -Q && cd -'
alias paper_view='cd $PAPER_PATH && evince paper.pdf & cd -'

# Baxter
alias baxter_shell="cd ~/catkin_ws && . baxter.sh && cd -"
alias baxter_enable="rosrun baxter_tools enable_robot.py -e"
alias baxter_disable="rosrun baxter_tools enable_robot.py -d"

# ROS Staff
EDITOR=geany
alias roscat="cd ~/catkin_ws && catkin_make && cd -"
source ~/catkin_ws/devel/setup.bash
#~ source ~/hr2_ws/devel/setup.bash
