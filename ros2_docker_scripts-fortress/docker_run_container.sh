#!/bin/bash

if [[ ( $@ == "--help") ||  $@ == "-h" ]]
then 
	echo "Usage: $0 [IMAGE_NAME] [CONTAINER_NAME] [FOLDER_NAME]"
else
	if [[ ($# -eq 0 ) ]]
	then 
		echo "Usage: $0 [IMAGE_NAME] [CONTAINER_NAME] [FOLDER_NAME]"
		#exit 0
	else
		dev_folder=$3
		local_folder="/home/$USER/$dev_folder"
		
		# Controlla se la directory esiste localmente
		if [ ! -d "$local_folder" ] 
		then
			echo "[WARNING] Development folder doesn't exist, creating a new one: $local_folder"
			mkdir -p "$local_folder"
		fi
		
		# Concedi accesso al server X11
		xhost +local:root

		# Esegui il container Docker
		IMAGE_ID=$1
		CONTAINER_NAME=$2
		docker run --privileged --rm -it \
  			--device=/dev/video0:/dev/video0 \
 		        --group-add video \
  			--name="$CONTAINER_NAME" \
  			--net=host \
  			--env="DISPLAY=$DISPLAY" \
  			--volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" \
  			--volume="$local_folder:/home/user/ros2_ws/src" \
  			"$IMAGE_ID"

		# Revoca accesso al server X11
		xhost -local:root
	fi 	
fi

