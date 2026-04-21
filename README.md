# 添加开机自启动
sudo cp potential-vision.service /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable potential-vision.service
sudo systemctl start potential-vision.service

# 删除开机自启动
sudo systemctl stop potential-vision.service
sudo systemctl disable potential-vision.service
sudo rm /etc/systemd/system/potential-vision.service

# 停止开机自启动
sudo systemctl disable potential-vision.service

# 手动开启/停止/重启
sudo systemctl start potential-vision.service
sudo systemctl stop potential-vision.service
sudo systemctl restart potential-vision.service

# 查看服务日志
sudo systemctl status potential-vision.service
sudo journalctl -u potential-vision.service -b
