# vendor_hand
wire vendor inspired robot hand

### setup
```bash
git clone git@github.com:hrjp/vendor_hand.git
sudo apt update 
rosdep install -i -y --from-paths src
catkin build
```
### UR setup
```bash
sudo apt update
sudo apt install -y python3-pip
pip install --user ur_rtde
```