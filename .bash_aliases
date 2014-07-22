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

# Custom Aliases
alias up='cd ..'
alias up2='up && up'
alias up3='up2 && up'
alias up4='up3 && up'
alias cd_catkin='cd ~/catkin_ws/src'
alias killgazebo="killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient"

# Tesis
PHD_THESIS_PATH=~/git/phd-thesis
function phd_labels() {
  find $PHD_THESIS_PATH -exec grep -oP '(?<=\\label\{)(.*?)(?=\})' {} \; | sort; }
function phd_cites() {
  find $PHD_THESIS_PATH -exec grep -oP '(?<=\\cite\{)(.*?)(?=\})' {} \; | sort; }
function phd_log() {
  LOG_FILE=$PHD_THESIS_PATH/thesis.log
  grep -rn 'LaTeX Warning:' $LOG_FILE;
  grep -rn '! ' $LOG_FILE; }
alias phd_build='cd $PHD_THESIS_PATH && scons -Q && cd -'

# Journal
JOURNAL_PATH=~/git/kinematics-journal
function journal_labels() {
  find $JOURNAL_PATH \( -path $JOURNAL_PATH/doc -prune -o -name '*.tex' \) -exec grep -oPnH '(?<=\\label\{)(.*?)(?=\})' {} \; | sort; }
function journal_cites() {
  find $JOURNAL_PATH \( -path $JOURNAL_PATH/doc -prune -o -name '*.tex' \) -exec grep -oPnH '(?<=\\cite\{)(.*?)(?=\})' {} \; | sort; }
function journal_log() {
  LOG_FILE=$JOURNAL_PATH/paper.log
  grep -rn 'LaTeX Warning:' $LOG_FILE;
  grep -rn '! ' $LOG_FILE; }
alias journal_build='cd $JOURNAL_PATH && scons -Q && cd -'

# Baxter
alias baxter_shell="cd ~/catkin_ws && . baxter.sh && cd -"

# ROS Staff
EDITOR=geany
alias roscat="cd ~/catkin_ws && catkin_make && cd -"
source ~/catkin_ws/devel/setup.bash
#~ source ~/hr2_ws/devel/setup.bash
