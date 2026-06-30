#hf auth login --token ${HUGGINGFACE_TOKEN} --add-to-git-credential
#HF_USER=$(NO_COLOR=1 hf auth whoami | awk -F': *' 'NR==1 {print $2}')
#echo $HF_USER


DATASET_ROOT=$HOME/lerobot_recordings
#DATASET_REPO_ID=${HF_USER}/record-test
DATASET_REPO_ID=./record-test


lerobot-replay \
    --robot.type=lerobot_robot_ur \
    --dataset.repo_id=$DATASET_REPO_ID \
    --dataset.root=$DATASET_ROOT \
    --dataset.episode=0
