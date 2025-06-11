- play test mcap:`ros2 bag play /home/heng.li/repo/vlm_ws/src/vita_agent/data/20250527_0.mcap` 
or with --loop
- test all:`python3 -m pytest test/ -v`
- test with ignore: `python3 -m pytest src/vita_agent/test/ -v --ignore=src/vita_agent/test/test_copyright.py`