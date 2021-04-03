/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include <linux/proc_fs.h>

//#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define SP0A28_P0_0xdd	0x80
#define SP0A28_P0_0xde	0xb0 //80
//sharpness                         

#define SP0A28_P1_0xe8         0x20 //0x10//10//;sharp_fac_pos_outdoor    �ױ�
#define SP0A28_P1_0xec         0x14//0x18 //0x20//20//;sharp_fac_neg_outdoor    �ڱ�
#define SP0A28_P1_0xe9         0x18 //0x0a//0a//;sharp_fac_pos_nr             �ױ�
#define SP0A28_P1_0xed         0x0c//0x10 //0x20//20//;sharp_fac_neg_nr            �ڱ�
#define SP0A28_P1_0xea         0x18 //0x08//08//;sharp_fac_pos_dummy    �ױ�
#define SP0A28_P1_0xef          0x0c//0x10 //0x18//18//;sharp_fac_neg_dummy    �ڱ�
#define SP0A28_P1_0xeb         0x10 //0x08//08//;sharp_fac_pos_low           �ױ�
#define SP0A28_P1_0xf0          0x0c//0x10 //0x08//18//;sharp_fac_neg_low           �ڱ�
 

//saturation
#define SP0A28_P0_0xd3	0x8c //9e
#define SP0A28_P0_0xd4	0x8c //9e
#define SP0A28_P0_0xd6  0x7c //80
#define SP0A28_P0_0xd7  0x60 //70
#define SP0A28_P0_0xd8	0x8c
#define SP0A28_P0_0xd9	0x8c
#define SP0A28_P0_0xda  0x7c
#define SP0A28_P0_0xdb  0x60
                               

//Ae target

#define SP0A28_P0_0xf7   0x78//0x80

#define SP0A28_P0_0xf8   0x70//0x78

#define SP0A28_P0_0xf9	0x78//0x78
#define SP0A28_P0_0xfa	0x70//0x70

#define SP0A28_SENSOR_NAME "sp0a28"
DEFINE_MSM_MUTEX(sp0a28_mut);

static struct msm_sensor_ctrl_t sp0a28_s_ctrl;

static struct msm_sensor_power_setting sp0a28_power_setting[] = {
  #if 0
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},

	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 10,
	},

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 10,
	},

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 10,
	},
	
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
  #else
      {
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},	
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
  #endif
};

static struct msm_camera_i2c_reg_conf sp0a28_start_settings[] = {
	{0xfd,0x00},
	{0x2f,0x01},
	{0xfd,0x01},
	{0x6b,0x00},
	{0xfd,0x00},
	{0x36,0x00},
	{0xfd,0x00},
};

static struct msm_camera_i2c_reg_conf sp0a28_stop_settings[] = {
	{0xfd,0x00},
	{0x36,0xa0},
	{0xe7,0x03},
	{0xe7,0x00},
	{0xfd,0x01},		
	{0x6b,0x01},
	{0xfd,0x00}, 
	{0x0e,0x00},
  	{0xfd,0x00},
};

 

static struct msm_camera_i2c_reg_conf sp0a28_recommend_settings[] = {
	{0xfd,0x00},
	{0x36,0xa0},
	{0xe7,0x03},
	{0xe7,0x00},
	{0xfd,0x01},
	{0x69,0x07},
	{0x78,0xff},
	{0x79,0xff},
	{0x7c,0x6c},
	{0x71,0x02},
	{0x72,0x09},
	{0x6c,0x02},
	{0x6d,0x03},
	{0x6e,0x05},
	{0x7d,0x01},
	{0xfd,0x00},
	{0x32,0x00},
	{0xe7,0x03},
	{0xe7,0x00},
	{0x0f,0x40},
	{0x10,0x40},
	{0x11,0x10},
	{0x12,0xa0},
	{0x13,0xff},
	{0x14,0x18},
	{0x15,0x00},
	{0x16,0x08},
	{0x1a,0x37},
	{0x1b,0x17},
	{0x1c,0x2f},
	{0x1d,0x00},
	{0x1e,0x57},
	{0x21,0x34},//f
	{0x22,0x12},
	{0x24,0x80},
	{0x25,0x02},
	{0x26,0x03},
	{0x27,0xeb},
	{0x28,0x5f},
	{0x5f,0x02},
	{0xfb,0x33},

	//blacklevel
	{0x65,0x18},//18
	{0x66,0x18},//18
	{0x67,0x18},//18
	{0x68,0x18},//18
	//ae setting
	//50Hz_24MHz_19~10fps           
	{0xfd,0x00},                     
	{0x03,0x01},                     
	{0x04,0x1d},                     
	{0x05,0x00},                     
	{0x06,0x00},                     
	{0x09,0x01},//                     
	{0x0a,0x93},//                           
	{0xf0,0x5f},                           
	{0xf1,0x00},                           
	{0xfd,0x01},                           
	{0x90,0x0a},                           
	{0x92,0x01},                           
	{0x98,0x5f},                           
	{0x99,0x00},                     
	{0x9a,0x03},                     
	{0x9b,0x00},                                       
	{0xfd,0x01},                           
	{0xce,0xb6},                           
	{0xcf,0x03},                           
	{0xd0,0xb6},                           
	{0xd1,0x03},                     
	{0xfd,0x00},    

	{0xfd,0x01},
	{0xc4,0x6c},
	{0xc5,0x7c},
	{0xca,0x30},
	{0xcb,0x45},
	{0xcc,0x60},
	{0xcd,0x60},

	//DP
	{0xfd,0x00},
	{0x45,0x00},
	{0x46,0x99},
	{0x79,0xff},
	{0x7a,0xff},
	{0x7b,0x10},
	{0x7c,0x10},

	//140117 up
	{0xfd,0x01},
	{0x35,0x14},
	{0x36,0x14},
	{0x37,0x1c},
	{0x38,0x1c},
	{0x39,0x10},
	{0x3a,0x10},
	{0x3b,0x1a},
	{0x3c,0x1a},
	{0x3d,0x10},
	{0x3e,0x10},
	{0x3f,0x15},
	{0x40,0x20},
	{0x41,0x0a},
	{0x42,0x08},
	{0x43,0x08},
	{0x44,0x0a},
	{0x45,0x00},
	{0x46,0x00},
	{0x47,0x00},
	{0x48,0xfd},
	{0x49,0x00},
	{0x4a,0x00},
	{0x4b,0x04},
	{0x4c,0xfd},
	{0xfd,0x00},
	{0xa1,0x20},
	{0xa2,0x20},
	{0xa3,0x20},
	{0xa4,0xff},

	//smooth
	{0xfd,0x01},
	{0xde,0x0f},
	{0xfd,0x00},
	//��ͨ����ƽ����ֵ   

	{0x57,0x08},//raw_dif_thr_outdoor
	{0x58,0x0e},//raw_dif_thr_normal 
	{0x56,0x10},//raw_dif_thr_dummy 
	{0x59,0x10},//raw_dif_thr_lowlight

	//GrGbƽ����ֵ
	{0x89,0x08},//raw_grgb_thr_outdoor
	{0x8a,0x0e},//raw_grgb_thr_normal 
	{0x9c,0x10},//raw_grgb_thr_dummy  
	{0x9d,0x10},//raw_grgb_thr_lowlight

	//Gr\Gb֮��ƽ��ǿ��
	{0x81,0xd8},//raw_gflt_fac_outdoor
	{0x82,0x98},//raw_gflt_fac_normal
	{0x83,0x80},//raw_gflt_fac_dummy
	{0x84,0x80},//raw_gflt_fac_lowlight

	//Gr��Gb��ͨ����ƽ��ǿ�� 
	{0x85,0xd8},//raw_gf_fac_outdoor 
	{0x86,0x98},//raw_gf_fac_normal 
	{0x87,0x80},//raw_gf_fac_dummy  
	{0x88,0x80},//raw_gf_fac_lowlight

	//R��Bƽ��ǿ�� 
	{0x5a,0xff},//raw_rb_fac_outdoor
	{0x5b,0xb8},//40//raw_rb_fac_normal
	{0x5c,0xa0},//raw_rb_fac_dummy
	{0x5d,0xa0},//raw_rb_fac_lowlight

	//adt ƽ����ֵ����Ӧ
	{0xa7,0xff},
	{0xa8,0xff},//0x2f
	{0xa9,0xff},//0x2f
	{0xaa,0xff},//0x2f

	//dem_morie_thr ȥĦ����
	{0x9e,0x10},

	//sharpen
	{0xfd,0x01},
	{0xe2,0x30},//sharpen_y_base
	{0xe4,0xa0},//sharpen_y_max

	{0xe5,0x08},//rangek_neg_outdoor
	{0xd3,0x10},//rangek_pos_outdoor  
	{0xd7,0x08},//range_base_outdoor  

	{0xe6,0x08},//rangek_neg_normal
	{0xd4,0x10},//rangek_pos_normal
	{0xd8,0x08},//range_base_normal 

	{0xe7,0x10},//rangek_neg_dummy
	{0xd5,0x10},//rangek_pos_dummy
	{0xd9,0x10},//range_base_dummy

	{0xd2,0x10},//rangek_neg_lowlight
	{0xd6,0x10},//rangek_pos_lowlight
	{0xda,0x10},//range_base_lowlight

	{0xe8,SP0A28_P1_0xe8},//0x20	//;sharp_fac_pos_outdoor
	{0xec,SP0A28_P1_0xec},//0x30	//;sharp_fac_neg_outdoor
	{0xe9,SP0A28_P1_0xe9},//0x10	//;sharp_fac_pos_nr
	{0xed,SP0A28_P1_0xed},//0x30	//;sharp_fac_neg_nr
	{0xea,SP0A28_P1_0xea},//0x10	//;sharp_fac_pos_dummy
	{0xef,SP0A28_P1_0xef},//0x20	//;sharp_fac_neg_dummy
	{0xeb,SP0A28_P1_0xeb},//0x10	//;sharp_fac_pos_low
	{0xf0,SP0A28_P1_0xf0},//0x20	//;sharp_fac_neg_low

	//CCM
	{0xfd,0x01},//01
	{0xa0,0x8a},//81
	{0xa1,0xfb},//f4
	{0xa2,0xfc},//0c
	{0xa3,0xfe},//0c
	{0xa4,0x93},//a6
	{0xa5,0xf0},//cd
	{0xa6,0x0b},//0c
	{0xa7,0xd8},//e0
	{0xa8,0x9d},//b3
	{0xa9,0x3c},//0c
	{0xaa,0x33},//30
	{0xab,0x0c},//0c
	{0xfd,0x00},//00


	//gamma      
	{0xfd,0x00},//00
	{0x8b,0x00},//0       
	{0x8c,0x0c},//12
	{0x8d,0x19},//1f
	{0x8e,0x2c},//31
	{0x8f,0x49},//4c
	{0x90,0x61},//62
	{0x91,0x77},//77
	{0x92,0x8a},//89
	{0x93,0x9b},//9b
	{0x94,0xa9},//a8
	{0x95,0xb5},//b5
	{0x96,0xc0},//c0
	{0x97,0xca},//ca
	{0x98,0xd4},//d4
	{0x99,0xdd},//dd
	{0x9a,0xe6},//e6
	{0x9b,0xef},//ef
	{0xfd,0x01},//01
	{0x8d,0xf7},//f7
	{0x8e,0xff},//ff
	{0xfd,0x00},//00

	//awb for 
	{0xfd,0x01},
	{0x28,0xc4},
	{0x29,0x9e},
	{0x11,0x13},
	{0x12,0x13},
	{0x2e,0x13},
	{0x2f,0x13},
	{0x16,0x1c},
	{0x17,0x1a},
	{0x18,0x1a},
	{0x19,0x54},
	{0x1a,0xa5},
	{0x1b,0x9a},
	{0x2a,0xef},	

	//rpc
	{0xfd,0x00},
	{0xe0,0x3a},//rpc_1base_max
	{0xe1,0x2c},//rpc_2base_max
	{0xe2,0x26},//rpc_3base_max
	{0xe3,0x22},//rpc_4base_max
	{0xe4,0x22},//rpc_5base_max
	{0xe5,0x20},//rpc_6base_max
	{0xe6,0x20},//rpc_7base_max
	{0xe8,0x20},//rpc_8base_max
	{0xe9,0x20},//rpc_9base_max
	{0xea,0x20},//rpc_10base_max
	{0xeb,0x1e},//rpc_11base_max
	{0xf5,0x1e},//rpc_12base_max
	{0xf6,0x1e},//rpc_13base_max

	//ae min gain 
	{0xfd,0x01},
	{0x94,0x60},//rpc_max_indr
	{0x95,0x1e},//rpc_min_indr
	{0x9c,0x60},//rpc_max_outdr
	{0x9d,0x1e},//rpc_min_outdr  

	//ae target
	{0xfd , 0x00},
	{0xed,SP0A28_P0_0xf7 + 0x04},//0x84
	{0xf7,SP0A28_P0_0xf7},		//0x80
	{0xf8,SP0A28_P0_0xf8},		//0x78
	{0xec,SP0A28_P0_0xf8 - 0x04},//0x74
	{0xef,SP0A28_P0_0xf9 + 0x04},//0x84
	{0xf9,SP0A28_P0_0xf9},		//0x80
	{0xfa,SP0A28_P0_0xfa},		//0x78
	{0xee,SP0A28_P0_0xfa - 0x04},//0x74

 

	///gray detect

	{0xfd,0x01},
	{0x30,0x40},
	{0x31,0x70},
	{0x32,0x20},
	{0x33,0xef},
	{0x34,0x02},
	{0x4d,0x40},
	{0x4e,0x15},
	{0x4f,0x13},

	///saturation

	{0xfd,0x00},
	{0xbe,0x5a},//0xaa
	{0xc0,0xff},
	{0xc1,0xff},

	{0xd3,SP0A28_P0_0xd3},
	{0xd4,SP0A28_P0_0xd4},
	{0xd6,SP0A28_P0_0xd6},
	{0xd7,SP0A28_P0_0xd7},
	{0xd8,SP0A28_P0_0xd8},
	{0xd9,SP0A28_P0_0xd9},
	{0xda,SP0A28_P0_0xda},
	{0xdb,SP0A28_P0_0xdb},


	///heq

	{0xfd,0x00},
	{0xdc,0x00},///heq_offset
	{0xdd,SP0A28_P0_0xdd},	//;ku
	{0xde,SP0A28_P0_0xde},// 220130114 0x98},///90///kl
	{0xdf,0x80},///heq_mean

	///YCnr

	{0xfd,0x00},
	{0xc2,0x08},   ///Ynr_thr_outdoor
	{0xc3,0x08},   ///Ynr_thr_normal
	{0xc4,0x08},   ///Ynr_thr_dummy
	{0xc5,0x10},   ///Ynr_thr_lowlight
	{0xc6,0x80},   ///cnr_thr_outdoor
	{0xc7,0x80},   ///cnr_thr_normal 
	{0xc8,0x80},   ///cnr_thr_dummy  
	{0xc9,0x80},   ///cnr_thr_lowlight 

	///auto lum

	{0xfd,0x00},
	{0xb2,0x18},///10 
	{0xb3,0x1f},
	{0xb4,0x30},//30
	{0xb5,0x45},//45


	///func enable

	{0xfd,0x00}, 
	{0x31,0x10},//0x70// yuxin modify for CTS test 2014.05.20
	{0x32,0x0d},
	{0x34,0x7e},///1e
	{0x33,0xef},//ef
	{0x35,0x40},  

};



static struct v4l2_subdev_info sp0a28_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order  = 0,
	},
};

static const struct i2c_device_id sp0a28_i2c_id[] = {
	{SP0A28_SENSOR_NAME, (kernel_ulong_t)&sp0a28_s_ctrl},
	{ }
};

static ssize_t sp0a28_camera_id_read_proc(char *page,char **start,off_t off,
	                                                int count,int *eof,void* data)
{		 	   
	int ret;	    
	unsigned char *camera_status = "FRONT Camera ID:Sp0a28 0.3M";  
	ret = strlen(camera_status);    
	sprintf(page,"%s\n",camera_status);	 	    
	return (ret + 1);
}
static void sp0a28_camera_proc_file(void)
{	   
	struct proc_dir_entry *proc_file  = create_proc_entry("driver/camera_id_front",0644,NULL);	    
	if(proc_file)     
	{		  	     
		proc_file->read_proc = sp0a28_camera_id_read_proc;			     
	}else     
	{       
	      pr_err(KERN_INFO "camera_proc_file error!\r\n");	    
	}
 }
static int32_t msm_sp0a28_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	CDBG("%s, E. ", __func__);

	return msm_sensor_i2c_probe(client, id, &sp0a28_s_ctrl);
}

static struct i2c_driver sp0a28_i2c_driver = {
	.id_table = sp0a28_i2c_id,
	.probe  = msm_sp0a28_i2c_probe,
	.driver = {
		.name = SP0A28_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client sp0a28_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static const struct of_device_id sp0a28_dt_match[] = {
	{.compatible = "qcom,sp0a28", .data = &sp0a28_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, sp0a28_dt_match);

static struct platform_driver sp0a28_platform_driver = {
	.driver = {
		.name = "qcom,sp0a28",
		.owner = THIS_MODULE,
		.of_match_table = sp0a28_dt_match,
	},
};

static int32_t sp0a28_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	CDBG("%s, E.", __func__);
	match = of_match_device(sp0a28_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	if (rc == 0)   
	{       
		sp0a28_camera_proc_file();    
	}   
	return rc;
}

static int __init sp0a28_init_module(void)
{
	int32_t rc;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&sp0a28_platform_driver,
		sp0a28_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&sp0a28_i2c_driver);
}

static void __exit sp0a28_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (sp0a28_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&sp0a28_s_ctrl);
		platform_driver_unregister(&sp0a28_platform_driver);
	} else
		i2c_del_driver(&sp0a28_i2c_driver);
	return;
}

int sp0a28_sensor_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		cdata->cfg.sensor_info.is_mount_angle_valid =
			s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
		cdata->cfg.sensor_info.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
		CDBG("%s:%d mount angle valid %d value %d\n", __func__,
			__LINE__, cdata->cfg.sensor_info.is_mount_angle_valid,
			cdata->cfg.sensor_info.sensor_mount_angle);

		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_camera_power_ctrl_t *p_ctrl;
		uint16_t size;
		int s_index = 0;
		if (copy_from_user(&sensor_slave_info,
				(void *)cdata->cfg.setting,
				sizeof(sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		/* Update sensor slave address */
		if (sensor_slave_info.slave_addr) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info.slave_addr >> 1;
		}

		/* Update sensor address type */
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;
		p_ctrl = &s_ctrl->sensordata->power_info;

		/* Update power up sequence */
		size = sensor_slave_info.power_setting_array.size;
		if (p_ctrl->power_setting_size < size) {
			struct msm_sensor_power_setting *tmp;
			tmp = kmalloc(sizeof(*tmp) * size, GFP_KERNEL);
			if (!tmp) {
				pr_err("%s: failed to alloc mem\n", __func__);
				rc = -ENOMEM;
				break;
			}
			kfree(p_ctrl->power_setting);
			p_ctrl->power_setting = tmp;
		}
		p_ctrl->power_setting_size = size;


		rc = copy_from_user(p_ctrl->power_setting, (void *)
			sensor_slave_info.power_setting_array.power_setting,
			size * sizeof(struct msm_sensor_power_setting));
		if (rc) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(sensor_slave_info.power_setting_array.
				power_setting);
			rc = -EFAULT;
			break;
		}
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (s_index = 0; s_index <
			p_ctrl->power_setting_size; s_index++) {
			CDBG("%s i %d power up setting %d %d %ld %d\n",
				__func__,
				s_index,
				p_ctrl->power_setting[s_index].seq_type,
				p_ctrl->power_setting[s_index].seq_val,
				p_ctrl->power_setting[s_index].config_val,
				p_ctrl->power_setting[s_index].delay);
		}

		/* Update power down sequence */
		if (!sensor_slave_info.power_setting_array.power_down_setting ||
			0 == size) {
			pr_err("%s: Missing dedicated power down sequence\n",
				__func__);
			break;
		}
		size = sensor_slave_info.power_setting_array.size_down;

		if (p_ctrl->power_down_setting_size < size) {
			struct msm_sensor_power_setting *tmp;
			tmp = kmalloc(sizeof(*tmp) * size, GFP_KERNEL);
			if (!tmp) {
				pr_err("%s: failed to alloc mem\n", __func__);
				rc = -ENOMEM;
				break;
			}
			kfree(p_ctrl->power_down_setting);
			p_ctrl->power_down_setting = tmp;
		}
		p_ctrl->power_down_setting_size = size;


		rc = copy_from_user(p_ctrl->power_down_setting, (void *)
			sensor_slave_info.power_setting_array.
			power_down_setting,
			size * sizeof(struct msm_sensor_power_setting));
		if (rc) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(sensor_slave_info.power_setting_array.
				power_down_setting);
			rc = -EFAULT;
			break;
		}
		for (s_index = 0; s_index <
			p_ctrl->power_down_setting_size; s_index++) {
			CDBG("%s i %d power DOWN setting %d %d %ld %d\n",
				__func__,
				s_index,
				p_ctrl->power_down_setting[s_index].seq_type,
				p_ctrl->power_down_setting[s_index].seq_val,
				p_ctrl->power_down_setting[s_index].config_val,
				p_ctrl->power_down_setting[s_index].delay);
		}

		break;
	}

	case CFG_SET_INIT_SETTING:
		/* Write Recommend settings */
		CDBG("%s, sensor write init setting!!\n", __func__);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			sp0a28_recommend_settings,
			ARRAY_SIZE(sp0a28_recommend_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		break;
		
	case CFG_SET_RESOLUTION:
		break;
		
	case CFG_SET_STOP_STREAM:
		CDBG("%s, sensor stop stream!!\n", __func__);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			sp0a28_stop_settings,
			ARRAY_SIZE(sp0a28_stop_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		break;
		
	case CFG_SET_START_STREAM:
		CDBG("%s, sensor start stream!!\n", __func__);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			sp0a28_start_settings,
			ARRAY_SIZE(sp0a28_start_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		break;	
		
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
	}
	case CFG_SLAVE_READ_I2C: {
		struct msm_camera_i2c_read_config read_config;
		uint16_t local_data = 0;
		uint16_t orig_slave_addr = 0, read_slave_addr = 0;
		if (copy_from_user(&read_config,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_read_config))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		read_slave_addr = read_config.slave_addr;
		CDBG("%s:CFG_SLAVE_READ_I2C:", __func__);
		CDBG("%s:slave_addr=0x%x reg_addr=0x%x, data_type=%d\n",
			__func__, read_config.slave_addr,
			read_config.reg_addr, read_config.data_type);
		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				read_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				read_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
				__func__, orig_slave_addr,
				read_slave_addr >> 1);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,
				read_config.reg_addr,
				&local_data, read_config.data_type);
		if (rc < 0) {
			pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
			break;
		}
		if (copy_to_user((void __user *)read_config.data,
			(void *)&local_data, sizeof(uint16_t))) {
			pr_err("%s:%d copy failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		break;
	}
	case CFG_SLAVE_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_array_write_config write_config;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		uint16_t write_slave_addr = 0;
		uint16_t orig_slave_addr = 0;

		if (copy_from_user(&write_config,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_array_write_config))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:CFG_SLAVE_WRITE_I2C_ARRAY:", __func__);
		CDBG("%s:slave_addr=0x%x, array_size=%d\n", __func__,
			write_config.slave_addr,
			write_config.conf_array.size);

		if (!write_config.conf_array.size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		reg_setting = kzalloc(write_config.conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting,
				(void *)(write_config.conf_array.reg_setting),
				write_config.conf_array.size *
				sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		write_config.conf_array.reg_setting = reg_setting;
		write_slave_addr = write_config.slave_addr;
		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				write_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				write_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
				__func__, orig_slave_addr,
				write_slave_addr >> 1);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &(write_config.conf_array));
		if (s_ctrl->sensor_i2c_client->cci_client) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				orig_slave_addr;
		} else if (s_ctrl->sensor_i2c_client->client) {
			s_ctrl->sensor_i2c_client->client->addr =
				orig_slave_addr;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_DOWN) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_up) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 1);

			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %ld\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
			pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;

	case CFG_POWER_DOWN:
		kfree(s_ctrl->stop_setting.reg_setting);
		s_ctrl->stop_setting.reg_setting = NULL;
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_down) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 0);

			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %ld\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(stop_setting,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;

		if (!stop_setting->size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
			(void *)reg_setting,
			stop_setting->size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}
	case CFG_SET_SATURATION: {

		break;
	}
	case CFG_SET_CONTRAST: {

		break;
	}
	case CFG_SET_SHARPNESS: {

		break;
	}
	case CFG_SET_ISO: {

		break;
	}
	case CFG_SET_EXPOSURE_COMPENSATION: {

		break;
	}
	case CFG_SET_EFFECT: {

		break;
	}
	case CFG_SET_ANTIBANDING: {

		break;
	}
	case CFG_SET_BESTSHOT_MODE: {

		break;
	}
	case CFG_SET_WHITE_BALANCE: {

		break;
	}
	case CFG_SET_AUTOFOCUS: {
			break;
	}
	case CFG_CANCEL_AUTOFOCUS: {
		break;
	}
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}


int32_t sp0a28_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;

	CDBG("%s,  E. calling i2c_read:, i2c_addr:0x%x, id_reg_addr:0x%x",
		__func__,
		s_ctrl->sensordata->slave_info->sensor_slave_addr,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x02,
			&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}
	CDBG("%s: read  id: %x ; expected id: 0xa2\n", __func__, chipid);
	if (chipid != 0xa2) {
		pr_err("msm_sensor_match_id chip id doesnot match \n");
		return -ENODEV;
	}

	return rc;
}


static struct msm_sensor_fn_t sp0a28_sensor_func_tbl = {
	.sensor_config = sp0a28_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = sp0a28_match_id,
};

static struct msm_sensor_ctrl_t sp0a28_s_ctrl = {
	.sensor_i2c_client = &sp0a28_sensor_i2c_client,
	.power_setting_array.power_setting = sp0a28_power_setting,
	.power_setting_array.size = ARRAY_SIZE(sp0a28_power_setting),
	.msm_sensor_mutex = &sp0a28_mut,
	.sensor_v4l2_subdev_info = sp0a28_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(sp0a28_subdev_info),
	.func_tbl = &sp0a28_sensor_func_tbl,
};

module_init(sp0a28_init_module);
module_exit(sp0a28_exit_module);
MODULE_DESCRIPTION("0.3MP YUV sensor driver");
MODULE_LICENSE("GPL v2");
