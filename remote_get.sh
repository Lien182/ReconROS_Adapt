DATE=`date +%d-%m-%y_%H-%M`
NAME=${PWD##*/}
rsync -r -v clienen@cc-9.cs.upb.de:/tmp/christian/hw_build_${NAME} hw_build_remote_${DATE}
