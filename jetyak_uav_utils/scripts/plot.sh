# Run on client machine

ssh ubuntu@$1 "./git/jetyak_uav/jetyak_uav_utils/scripts/updateLatest.sh"
scp ubuntu@$1:~/git/jetyak_uav/jetyak_uav_utils/scripts/latest.ignore .
scp ubuntu@$1:/media/ubuntu/Manifold/FlightLog/`cat latest.ignore` .
python plotter.py $(ls -rt . | tail -n1)
