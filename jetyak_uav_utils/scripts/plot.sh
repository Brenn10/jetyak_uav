# Run on client machine

ssh ubuntu@10.34.7.41 ".//media/ubuntu/Manifold/FlightLog/updateLatest.sh"
scp -r ubuntu@10.34.7.41:/media/ubuntu/Manifold/FlightLog/latest .
python plotter.py $(ls -rt target-directory | tail -n1)
