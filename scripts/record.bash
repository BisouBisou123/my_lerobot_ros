#hf auth login --token ${HUGGINGFACE_TOKEN} --add-to-git-credential
#HF_USER=$(NO_COLOR=1 hf auth whoami | awk -F': *' 'NR==1 {print $2}')
#echo $HF_USER


DATASET_ROOT=$HOME/lerobot_recordings
#DATASET_REPO_ID=${HF_USER}/record-test
DATASET_REPO_ID=./record-test

lerobot-record \
    --robot.type=lerobot_robot_ur \
    --robot.cameras="{ front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}}" \
    --dataset.num_episodes=5 \
    --dataset.episode_time_s=15 \
    --dataset.single_task="Grab the black cube" \
    --dataset.streaming_encoding=true \
    --teleop.type=lerobot_teleoperator_teachbot \
    --dataset.vcodec=h264 \
    --dataset.push_to_hub=False \
    --dataset.repo_id=$DATASET_REPO_ID \
    --dataset.root=$DATASET_ROOT \
    --display_data=false \
    --dataset.encoder_threads=4 \
    --robot.ros2_interface.sim=true
    # --dataset.vcodec=auto \
    #--dataset.encoder_threads=2
    # --dataset.vcodec=auto \
    # --dataset.repo_id=${HF_USER}/record-test \

