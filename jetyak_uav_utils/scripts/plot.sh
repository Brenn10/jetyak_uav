# Run on client machine

ssh ubuntu@10.34.7.41 "./git/jetyak_uav/jetyak_uav_utils/scripts/updateLatest.sh"
scp -r ubuntu@10.34.7.41:~/git/jetyak_uav/jetyak_uav_utils/scripts/latest .
python plotter.py $(ls -rt . | tail -n1)
