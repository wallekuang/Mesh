# Mesh

这个Demo是基于ST BLE mesh 8个字节实现分包重组的演示demo。
备注: 代码仅供参考，使用需要根据实际调试测试。

Keil 或者 IAR工程路径:
工程路径:Firmware\Projects\BlueNRG-2\Applications\Lighting_Demo 
如果使用BlueNRG-1，需要自行添加文件编译。
备注: 代码中默认在发送数据时使用了动态内存，ST官方的IAR工程默认无法申请
动态内存，需要首先分配内存到堆。
	需要在 *.icf文件中添加类似这样的语句手工分配内存给堆先。
	place in REGION_RAM 	{block HEAP };
	define block HEAP     with alignment = 8, size = 0x400  { };

测试步骤:
	1. 编译烧录好 两块EVB板子(STEVAL-IDB008V2)
	2. 使用手机provision 这两设备成节点。（手机建议使用Android手机，请使用最新版本的app进行测试，当前最新APP为1.10.000）
		可以使用Google Play下载 或者在安装mesh SDK的目录下找到相关的apk安装:
		参考路径  C:\Users\你的用户名\ST\STSW-BNRG-Mesh_1.08.000\Android\apk
		SDK 官网链接: https://www.st.com/content/st_com/en/products/embedded-software/wireless-connectivity-software/stsw-bnrg-mesh.html
		
	3. 配置两个节点。将节点A  publish 地址设置为节点B ， 节点B  publish 地址设置为节点A。
	4. 点击 PUSH1  按键 ， 默认发送200字节的数据 到另一个节点。
	
	
	
代码主要更改:
	a. 主要公共服务型文件
		double_list.h
		dcache.c dcache.h
	b. 发送和接收协议文件
		sender.c  sender.h
		receiver.c receiver.h
	c. 应用或者测试文件
		app_control.c app_control.
	