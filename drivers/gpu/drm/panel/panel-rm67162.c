/*
 * Copyright © 2016 K Quigley Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Portions of this file (derived from panel-simple.c) are:
 *
 * Copyright (C) 2013, NVIDIA Corporation.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/**
 * DOC: Raspberry Pi 1.2" round OLED panel driver.
 *
 *
 * This driver presents this device as a MIPI DSI panel to the DRM
 * driver, and in time a seperate modle will the touchscreen as a HID device.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>

#include <video/display_timing.h>

#include <drm/drm_panel.h>
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>



#define DSI_DRIVER_NAME "gvo-ts-dsi"

// define this for extra kernel prints to check driver function code flow
//#define TRACE

/* Steps to initial display */
/* List of RM67162 used commands  (MIPI)                                */
/* Detailed in RM67162 Data Sheet 'DATA_SHEET_RM67162_V0.0_20160516.pdf' */
/* Version 0.0 of May 16, 2016                                         */
#define  RM67162_CMD_NOP                    0x00   /* NOP command      */
#define  RM67162_CMD_SWRESET                0x01  /* Sw reset command */
#define  RM67162_CMD_RDDID                  0x04  /* Sw reset command */
#define  RM67162_CMD_RDNUMED                0x05  /* Read Number of Errors on DSI  */
#define  RM67162_CMD_RDDPM                  0x0A  /* Read Display Power Mode - MIPI address Nancy*/
#define  RM67162_CMD_RDDMADCTR              0x0B  /* Read Display MADCTR */
#define  RM67162_CMD_RDDCOLMOD              0x0C  /* Read Display Pixel Format */
#define  RM67162_CMD_RDDIM                  0x0D  /* Read Display Image Mode */
#define  RM67162_CMD_RDDSM                  0x0E  /* Read Display Signal Mode */
#define  RM67162_CMD_RDDSDR                 0x0F  /* Read Display Self-Diagnostic Result */
#define  RM67162_CMD_SLPIN                  0x10  /* Sleep In */
#define  RM67162_CMD_SLPOUT                 0x11  /* Sleep Out */
#define  RM67162_CMD_PTLON                  0x12  /* Partial Display Mode On */
#define  RM67162_CMD_NORON                  0x13  /* Normal Display Mode On */
#define  RM67162_CMD_INVOFF                 0x20  /* Display Inversion Off */
#define  RM67162_CMD_INVON                  0x21  /* Display Inversion On */
#define  RM67162_CMD_ALLPOFF                0x22  /* All Pixel Off */
#define  RM67162_CMD_ALLPON                 0x23  /* All Pixel On */
#define  RM67162_CMD_DISPOFF                0x28  /* Display Off */
#define  RM67162_CMD_DISPON                 0x29  /* Display on */
#define  RM67162_CMD_CASET                  0x2A  /* Set Column Start Address */
#define  RM67162_CMD_RASET                  0x2B  /* Set Row Start Address */
#define  RM67162_CMD_RAMWR                  0x2C  /* Memory Write */
#define  RM67162_CMD_RAMRD                  0x2E  /* Memory Read */
#define  RM67162_CMD_PTLAR_H                0x30  /* Horizontal Partial Area */
#define  RM67162_CMD_PTLAR_V                0x31  /* Vertical Partial Area */
#define  RM67162_CMD_TEOFF                  0x34  /* Tearing Effect Line OFF */
#define  RM67162_CMD_TEON                   0x35  /* Tearing Effect Line ON */
#define  RM67162_CMD_MADCTR                 0x36  /* Scan Direction Control - MIPI address Nancy*/
#define  RM67162_CMD_IDMOFF                 0x38  /* Idle Mode Off - MIPI address Nancy*/
#define  RM67162_CMD_IDMON                  0x39  /* Enter idle mode - MIPI address Nancy*/
#define  RM67162_CMD_COLMOD                 0x3A  /* Interface Pixel Format */
#define  RM67162_CMD_RAMWRC                 0x3C  /* Memory Continuous Write */
#define  RM67162_CMD_RAMRDC                 0x3E  /* Memory Continuous Read */
#define  RM67162_CMD_STESL                  0x44  /* Set Tear Scanline */
#define  RM67162_CMD_GSL                    0x45  /* Get Scanline */
#define  RM67162_CMD_DSTBON                 0x4F  /* Deep Standby Mode On */
#define  RM67162_CMD_WRDISBV                0x51  /* Write Display Brightness - MIPI address Nancy*/
#define  RM67162_CMD_RDDISBV                0x52  /* Read Display Brightness */
#define  RM67162_CMD_WRCTRLD                0x53  /* Write Display Control */
#define  RM67162_CMD_RDCTRLD                0x54  /* Read Display Control */
#define  RM67162_CMD_WRRADACL               0x55  /* RAD ACL Control */
#define  RM67162_CMD_WRCE                   0x58  /* Set color enhance */
#define  RM67162_CMD_RDCE                   0x59  /* Read color enhance */
#define  RM67162_CMD_CESLRCTR_L             0x5A  /* Set color enhance1 Low Byte*/
#define  RM67162_CMD_CESLRCTR_H             0x5B  /* set color enhance1 High Byte*/
#define  RM67162_CMD_RDDDBS                 0xA1  /* Read DDB Start */
#define  RM67162_CMD_RDDDBC                 0xA8  /* Read DDB Continous */
#define  RM67162_CMD_RDFCS                  0xAA  /* Read First Checksum */
#define  RM67162_CMD_RDCCS                  0xAF  /* Read Continue Checksum */
#define  RM67162_CMD_SetHBMMode             0xB0  /* Set HBM Mode */
#define  RM67162_CMD_SetDSIMode             0xC2  /* Set DSI Mode */
#define  RM67162_CMD_SetDSPIMode            0xC4  /* Set DSPI Mode */
#define  RM67162_CMD_RDID1                  0xDA  /* ID1 Code: Manf */
#define  RM67162_CMD_RDID2                  0xDB  /* ID2 Code: Driver/Version  */
#define  RM67162_CMD_RDID3                  0xDC  /* ID3 Code: Driver/Version  */
#define  RM67162_CMD_MAUCCTR1               0xFE  /* Manufacture Command Set Control 1 - MIPI address, command switch  Nancy*/
#define  RM67162_CMD_MAUCCTR2               0xFF  /* Manufacture Command Set Control 2 */

#define RM67162_FORMAT_RGB888               ((uint32_t)0x77) /* Pixel format chosen is RGB888 : 24 bpp */
#define RM67162_FORMAT_RGB565               ((uint32_t)0x75) /* Pixel format chosen is RGB565 : 16 bpp */

#define RM67162_VERSION_ID                  (0x80) 

/* Variables for retry handling when sending setup commands to the panel */
static int failedCount;
static const int LOOP = 3;

const uint16_t manufacture_settings[] = {
//    0x01FE, // CMD_Status = 0x01     // Setting display controller address
    0x6206,    // DSC Read
    0x800E, // Display Signal mode read ?
    0x800F, // Display Selftest read ?
    0x7110, // Sleep In (takes no arguements)
    0x8113, // Normal DIsplay mode on  (takes no arguements)   RM67162_CMD_NORON
    0x8114, // Not in Datasheet
    0x8215, // Not in Datasheet
    0x8216, // Not in Datasheet
    0x8818, // Not in Datasheet
    0x5519, // Not in Datasheet
    0x101A, // Not in Datasheet
    0x991C, // Not in Datasheet
    0x031D, // Not in Datasheet
    0x031E, // Not in Datasheet
    0x031F, // Not in Datasheet
    0x0320, // Display Inversion off
    0x0325, // Not in Datasheet
    0x8D26, // Not in Datasheet
    0x032A, // Col Start
    0x8D2B, // Row Start
    0x0036, // Scan Direction   RM67162_CMD_MADCTR: 0xC0  (upside down)
    0x1037, // Not in Datasheet
    0x773A, // Pixel Interface format (Color Mode)  RM67162_CMD_COLMOD: RM67162_FORMAT_RGB888  (0x77 is default)
    0x003B, // Not in Datasheet
    0x203D,
    0x3A3F,
    0x3040,
    0x1A41,
    0x3342,
    0x2243,
    0x1144, // Tear Scan Line
    0x6645, // Read Scan line
    0x5546,
    0x4447,
    0x334C,
    0x224D,
    0x114E,
    0x664F, // Deep Standby
    0x5550,
    0x4451,
    0x3357,
    0x1B6B,
    0x5570,
    0x0C74
};

const uint16_t vgmp_settings[] = {
/* VGMP/VGSP Voltage Control */
//    0x02FE,	// Setting display controller address
    0x409B,
    0x009C,
    0x209D
};

const uint16_t vgsp_settings[] = {
/* VGMP/VGSP Voltage Control */
//    0x03FE,   // Setting display controller address
    0x409B,
    0x009C,
    0x209D
};

const uint16_t vsr_settings[] = {
/* VSR Command */
//    0x04FE,
    0x105D
};

const uint16_t vsr1_settings[] = {
/* VSR1 Timing Set */
//    0x04FE,       // Setting display controller address
    0x8D00,
    0x0001,
    0x0102,
    0x0103,
    0x1004,
    0x0105,
    0xA706,
    0x2007,
    0x0008
};

const uint16_t vsr2_settings[] = {
/* VSR2 Timing Set */
//    0x04FE,       // Setting display controller address
    0xC209,
    0x000A,
    0x020B,
    0x010C,
    0x400D,
    0x060E,
    0x010F,
    0xA710,
    0x0011
};

const uint16_t vsr3_settings[] = {
/* VSR3 Timing Set */
//    0x04FE,       // Setting display controller address
    0xC212,
    0x0013,
    0x0214,
    0x0115,
    0x4016,
    0x0717,
    0x0118,
    0xA719,
    0x001A
};

const uint16_t vsr4_settings[] = {
/* VSR4 Timing Set */
//    0x04FE,       // Setting display controller address
    0x821B,
    0x001C,
    0xFF1D,
    0x051E,
    0x601F,
    0x0220,
    0x0121,
    0x7C22,
    0x0023
};

const uint16_t vsr5_settings[] = {
/* VSR5 Timing Set */
//    0x04FE,       // Setting display controller address
    0xC224,
    0x0025,
    0x0426,
    0x0227,
    0x7028,
    0x0529,
    0x742A,
    0x8D2B,
    0x002D
};

const uint16_t vsr6_settings[] = {
/* VSR6 Timing Set */
//    0x04FE,       // Setting display controller address
    0xC22F,
    0x0030,
    0x0431,
    0x0232,
    0x7033,
    0x0734,
    0x7435,
    0x8D36,
    0x0037
};

const uint16_t vsrMap_settings[] = {
/* VSR Marping command  */
//    0x04FE,       // Setting display controller address
    0x205E,
    0x315F,
    0x5460,
    0x7661,
    0x9862
};

const uint16_t elvss_settings[] = {
/* ELVSS VOLTAGE SET */
//    0x05FE,       // Setting display controller address
    0x0505,
    0x042A,
    0x0091
};

const uint16_t set_memory_address_settings[] = {
/* VSR Command */
//    0x00FE,       // Setting display controller address
    0x0035
};

const uint16_t set_cmd1_settings[] = {
/* After essential initial, send addition for fwip */
//    0x00FE,       // Setting display controller address
    0xff51, // SET brightness of backlight 
    0xC036  // set Scan direction
};

struct panel_desc {
    const struct drm_display_mode *modes;
    unsigned int num_modes;
    const struct display_timing *timings;
    unsigned int num_timings;

    unsigned int bpc;

    /**
     * @width: width (in millimeters) of the panel's active display area
     * @height: height (in millimeters) of the panel's active display area
     */
    struct {
        unsigned int width;
        unsigned int height;
    } size;

    /**
     * @prepare: the time (in milliseconds) that it takes for the panel to
     *           become ready and start receiving video data
     * @enable: the time (in milliseconds) that it takes for the panel to
     *          display the first valid frame after starting to receive
     *          video data
     * @disable: the time (in milliseconds) that it takes for the panel to
     *           turn the display off (no content is visible)
     * @unprepare: the time (in milliseconds) that it takes for the panel
     *             to power itself down completely
     */
    struct {
        unsigned int prepare;
        unsigned int enable;
        unsigned int disable;
        unsigned int unprepare;
        unsigned int reset;
        unsigned int power_on;
    } delay;

    u32 bus_format;
    u32 bus_flags;
};

struct rm67162_variant {
    u8 manafacturer;
    u8 version;
};


struct panel_rm67162 {
    struct drm_panel base;
    struct mipi_dsi_device *dsi;

    // from panel simple
    bool prepared;
    bool enabled;

    const struct panel_desc *desc;

    struct backlight_device *backlight;
    struct regulator_bulk_data supplies[2];
    struct i2c_adapter *ddc;

    struct gpio_desc *enable_gpio;
    struct gpio_desc *reset_gpio;
    int error;
    u8 version;
    u8 id;

    const struct rm67162_variant *variant;

};

// Version and Manfacturer ID codes we support
static const struct rm67162_variant rm67162_variants[] = {
    {
        .manafacturer = 0,
        .version = 128,
    }, {
        .manafacturer = 0,
        .version = 210,
    }
};


static const struct drm_display_mode gov_rm67162_mode[] = {
    {
    .clock = 5800,
    .hdisplay = 390,
    .hsync_start = 390 + 12,     // hdisplay + h_front_porch
    .hsync_end = 390 + 12 + 12,  // hsync_start + h_back_porch
    .htotal = 390 + 12 + 12 + 2, // hsync_end + h_pulse_width
    .vdisplay = 390,
    .vsync_start = 390 + 12,
    .vsync_end = 390 + 12 + 12,
    .vtotal = 390 + 12 + 12 + 2,
    .vrefresh = 30,
    },
};

struct panel_desc_dsi {
    struct panel_desc desc;

    unsigned long flags;
    enum mipi_dsi_pixel_format format;
    unsigned int lanes;
};

static const struct panel_desc_dsi gov_rm67162 = {
    .desc = {
        .modes = &gov_rm67162_mode[0],
        .num_modes = 1,
        .bpc = 8,
        .size = {
            .width = 30,
            .height = 30,
        },
        .delay = {
            .prepare = 190, //120,
            .enable = 200,
            .unprepare = 10,
            .disable = 100,
            .reset = 10,
            .power_on = 10,
        },
    },
    .flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM,
//| MIPI_DSI_CLOCK_NON_CONTINUOUS 
//| MIPI_DSI_MODE_VIDEO_SYNC_PULSE
    .format = MIPI_DSI_FMT_RGB888,
    .lanes = 1,
};


static const struct of_device_id dsi_of_match[] = {
    {
        .compatible = "gov,rm67162",
        .data = &gov_rm67162
    },
    { } /* sentinel */
};


static struct panel_rm67162 *panel_to_ts(struct drm_panel *panel)
{
    return container_of(panel, struct panel_rm67162, base);
}


static int rpi_dcs_write(struct panel_rm67162 *panel, const void *data, size_t len)
{
    int ret;
//      ret = mipi_dsi_dcs_write_buffer(panel->dsi, data, len);
      ret = mipi_dsi_generic_write(panel->dsi, data, len);
      if(ret<0)
        failedCount++;
      else
        return 0;
      return ret;
}

static int rpi_short_write(struct panel_rm67162 *panel, u8 cmd, u8 data, int sz)
{
    u8 msg[] = {cmd, data};
    return(rpi_dcs_write(panel, msg,sz));
}

static int rpi_dcs_read(struct panel_rm67162 *panel, u8 cmd, void *data, size_t len)
{
    int ret;
    int loop = LOOP; //3 possible have reading error

    while(loop-- > 0)
    {
      ret = mipi_dsi_dcs_read(panel->dsi, cmd, data, len);
//      ret = mipi_dsi_generic_read(panel->dsi, cmd, data, len);
      if (ret >= 0) 
          break;
      dev_err(panel->base.dev, "looped %d/%d, return %zd, ret: (%#x) reading seq: %*ph \n", LOOP-loop,LOOP, ret, cmd, (int)len, data);
    }

    return ret;
}

static int rm67162_write_cmd1(struct panel_rm67162 *p, u8 cmd, u8 data, int sz)
{
    int ret;
    int loop = LOOP;

    // Reading mode
    // Ensure panel is in CMD1- Mode
    while(loop-- > 0){
      ret = rpi_short_write(p,RM67162_CMD_MAUCCTR1, 0, 2);
      if (ret >= 0) {
        ret = rpi_short_write(p,  cmd, data, sz);  // 0xff read
        if(ret >= 0)
          return 0;
        dev_err(p->base.dev, "Writing cmd1 failed. looped %d/%d, ret %zd, writing seq %02x  data 0x%02x\n", LOOP-loop, LOOP, ret, cmd,data);
      }
  }
    return ret;
}

static int rm67162_set_page_mode(struct panel_rm67162 *p, u8 mode){

  return rpi_short_write(p,RM67162_CMD_MAUCCTR1, mode,2);
}

static int rm67162_read_mtp_id(struct panel_rm67162 *p)
{

    u8 id[6];
    int ret, i;
    int loop = LOOP;

    p->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

    // Reading:
    // Ensure panel is in CMD1- Mode
    while(loop-- >0){
      ret = rm67162_set_page_mode(p, 0); //
      if (ret >= 0) {
        ret = rpi_dcs_read(p,  RM67162_CMD_RDID1, id, 1);  // Manf ID
        ret = rpi_dcs_read(p,  RM67162_CMD_RDID2, &id[1], 1);  // Version ID
        ret = rpi_dcs_read(p,  RM67162_CMD_RDID3, &id[2], 1); // Driver ID

        dev_info(p->base.dev, "Display Panel ID: 0x%02x, 0x%02x, 0x%02x\n", id[0], id[1], id[2]);

        for (i = 0; i < ARRAY_SIZE(rm67162_variants); ++i) {
          if (id[1] == rm67162_variants[i].version)
            break;
        }
      }

      if (i >= ARRAY_SIZE(rm67162_variants)) {
        dev_err(p->base.dev, "found unsupported display version %d. It maybe mistake have another reading, looped %d/%d\n", id[1], LOOP-loop, LOOP);
        ret = -EINVAL;
        continue;
      }
      else
        break;
    }
    if(ret<0)
      return ret;

#ifdef TRACE
#if 0
    // read a couple of reset values
    ret = rpi_dcs_read(p,  RM67162_CMD_WRDISBV, &id[3], 1);  //0x51 write display brightness
    ret = rpi_dcs_read(p,  RM67162_CMD_MADCTR, &id[4], 1);  // 0x36 Scan Direction Control
    ret = rpi_dcs_read(p,  RM67162_CMD_RDDPM, &id[5], 1);  // 0x0A read display power mode
    dev_info(p->base.dev, "51 36 0A: 0x%02x, 0x%02x, 0x%02x\n", id[3], id[4], id[5]);
#endif
#endif // TRACE

    p->variant = &rm67162_variants[i];
    p->version = id[1];
    p->id = id[2];

    return 0; //ret;
}

#if 0
static int rm67162_check_values(struct drm_panel *panel)
{
    struct panel_rm67162 *p = panel_to_ts(panel);

    u8 id[6];
    int ret;
    int loop = LOOP;

    // Reading:
    // Ensure panel is in CMD1- Mode
    while(loop-- >0){
      ret = rm67162_set_page_mode(p, 0); //
      if (ret >= 0) {
        ret = rpi_dcs_read(p,  RM67162_CMD_WRDISBV, &id[3], 1);  //0x51 write display brightness
        ret = rpi_dcs_read(p,  RM67162_CMD_MADCTR, &id[4], 1);  // 0x36 Scan Direction Control
        ret = rpi_dcs_read(p,  RM67162_CMD_RDDPM, &id[5], 1);  // 0x0A read display power mode
        dev_info(panel->dev, "51 36 0A: 0x%02x, 0x%02x, 0x%02x\n", id[3], id[4], id[5]);
        if(ret>=0)
          break;
       }
      }

    return 0; //ret;
}
#endif

/*
* RM67162 Issue SW reset to LCD device
*
*/
static int rm67162_sw_reset (struct panel_rm67162 *p)
{
    int ret;
    //int loop = LOOP; // in case of failure. A few re-try to make sure reset being done.
    //bool swResetReady = false;

    ret = rm67162_write_cmd1(p, RM67162_CMD_SWRESET, 0,1);
    if (ret < 0) {
        dev_err(p->base.dev, " reset failed %zd \n", ret);
        return -EPROBE_DEFER;
        }

    if (p->desc->delay.enable) {
        msleep(p->desc->delay.enable);
        dev_err(p->base.dev, "post SW reset delay: %d\n",p->desc->delay.enable);
    }

    return 0;
}

#if 0
static int writeConfigLine( struct panel_rm67162 *p, uint16_t registerMode, uint16_t data)
{
  int ret;
  int loop = LOOP;
      // Reading mode
    // Ensure panel is in CMD1- Mode
    while(loop-- > 0){
      ret = rpi_short_write(p,0xff&registerMode, registerMode>>8, 2);
      if (ret >= 0) {
        ret = rpi_short_write(p,  0xff&data, data>>8, 2);
        if(ret >= 0)
          return 0;
        dev_err(p->base.dev, "Writing line failed. looped %d/%d, ret %zd, register 0x%04x data 0x%04x\n", LOOP-loop, LOOP, ret,registerMode,data);
      }
  }
    return ret;

}
#endif

static int writeConfigBlock( struct panel_rm67162 *p, const uint16_t *config, int size, uint16_t registerMode )
{
    int ret;
    u8 i=0, id;
    int loop = LOOP; //possible have writing error, a couple of more try again
    bool registerModeOn = false;
    int timeOutLine = -1;

  // start this block
  msleep(20);
  for (i=0; i<=size; i++)
  {

    while(loop-- > 0)
    {
      if(!registerModeOn) {
        ret = rpi_short_write(p,0xFF&registerMode, registerMode>>8,2);
        if (ret >= 0)
          registerModeOn = true;
        else
          dev_err(p->base.dev, "looped %d/%d, return %zd, entry: (%#x), *config 0x%04x \n", LOOP-loop,LOOP, ret, i, config[i]);
      }
      if (registerModeOn)
      {
        //ret = rpi_short_write(p,0xFF & *config, *config>>8,2);
        id = (i<size?i:size-1); // resend last data to test if the data being sucessfully send out
        ret = rpi_short_write(p,0xFF & config[id], config[id]>>8,2);
        if (ret >= 0)
          break;
        else
          {
          registerModeOn = false;
          dev_err(p->base.dev, "looped %d/%d, return %zd, entry: (%#x), *config 0x%04x \n", LOOP-loop,LOOP, ret, i, config[i]);
          if(i>0 && i>timeOutLine){  // the block is for resend previous data
            timeOutLine = i;
            i--; //=0;
            dev_alert(p->base.dev, "Will resend previous entry %d, at looped %d/%d, return %zd, config 0x%04x \n", i,LOOP-loop,LOOP, ret, config[i]);
          }
        }
      }
      if(loop == 5)
        msleep(50); // let it sleep 50ms if failed that bad
    }
    if (ret < 0) {
        //After LOOP times trial, init sequence still failed, defer the startup and try again
        ret = -EPROBE_DEFER;  // carry on, it may luckily doesn't matter
        }
    loop=LOOP;
  }

  return 0;
}

static int setMemoryAddress(struct panel_rm67162 *p)
{
    int ret;
    int loop = LOOP;
    bool registerModeOn = false;
    bool col2AIsDone = false;

    u8 col_2A[] = {RM67162_CMD_CASET,0,0x06,0x1,0x8B};	// {RM67162_CMD_CASET,0x02,0,0x87,0x1}; //{RM67162_CMD_CASET,0,0x02,0x1,0x87};
    u8 row_2B[] = {RM67162_CMD_RASET,0,0x02,0x1,0x87};	// {RM67162_CMD_RASET,0x08,0,0x8d,0x1};

    while(loop-- > 0)
    {
      if(!registerModeOn) {
        ret = writeConfigBlock(p,&set_memory_address_settings[0], ARRAY_SIZE(set_memory_address_settings),0x00FE);
        if (ret >= 0)
          registerModeOn = true;
      }
      if (registerModeOn)
      {
        if(!col2AIsDone) {
          ret = rpi_dcs_write(p, col_2A,5);
          if(ret<0) {
            registerModeOn = false;
            dev_err(p->base.dev, "looped %d/%d, return %zd, col_2A %*ph\n \n", LOOP-loop,LOOP, ret, 5, col_2A);
            continue;
          }
          else
            col2AIsDone = true;
        }
 
        ret = rpi_dcs_write(p, row_2B,5);
        if (ret < 0) {
          registerModeOn = false;
          dev_err(p->base.dev, "looped %d/%d, return %zd, col_2B %*ph\n", LOOP-loop,LOOP, ret, 5, row_2B);
        }
        else
          break;
      }
    }
    if (ret < 0) { //
      //Init sequence failed, defer the startup and try again
      ret = -EPROBE_DEFER;
    }

    return ret;
}

/*
* RM67162 Write manufacturer setting to LCD device
* Similar to panel-samsung-s6e8aa0.c s6e8aa0_set_sequence()
*
* Porting from lcd_init_rm67162 in Display_rm67162.c
*/
static int writeManufacturerSettings(struct drm_panel *panel)
{

   struct panel_rm67162 *p = panel_to_ts(panel);

   int ret;

   // Exiting Sleep mode re-sets the module's initial values !!!
   /* Step 1 SLEEP OUT */
   rpi_short_write(p, RM67162_CMD_SLPOUT, 0x0, 1);

   // Step 2 This 120 ms sleeping from specfication step.
   if (p->desc->delay.reset) {
        msleep(p->desc->delay.reset);    // Need to wait at least 5mS
   }

   /*
   manufacture Settings: Step 1-13
   */

   ret = writeConfigBlock(p,&manufacture_settings[0], ARRAY_SIZE(manufacture_settings),0x01FE);

   ret = writeConfigBlock(p,&vgmp_settings[0], ARRAY_SIZE(vgmp_settings),0x02FE);

   ret = writeConfigBlock(p,&vgsp_settings[0], ARRAY_SIZE(vgsp_settings),0x03FE);

   ret = writeConfigBlock(p,&vsr_settings[0], ARRAY_SIZE(vsr_settings),0x04FE);

   ret = writeConfigBlock(p,&vsr1_settings[0], ARRAY_SIZE(vsr1_settings),0x04FE);
   ret = writeConfigBlock(p,&vsr2_settings[0], ARRAY_SIZE(vsr2_settings),0x04FE);
   ret = writeConfigBlock(p,&vsr3_settings[0], ARRAY_SIZE(vsr3_settings),0x04FE);
   ret = writeConfigBlock(p,&vsr4_settings[0], ARRAY_SIZE(vsr4_settings),0x04FE);
   ret = writeConfigBlock(p,&vsr5_settings[0], ARRAY_SIZE(vsr5_settings),0x04FE);
   ret = writeConfigBlock(p,&vsr6_settings[0], ARRAY_SIZE(vsr6_settings),0x04FE);
   ret = writeConfigBlock(p,&vsrMap_settings[0], ARRAY_SIZE(vsrMap_settings),0x04FE);

   ret = writeConfigBlock(p,&elvss_settings[0], ARRAY_SIZE(elvss_settings),0x05FE);

   /* set memory address block */
   ret = setMemoryAddress(p);

   // brightness and scan direction:
   ret = writeConfigBlock(p,&set_cmd1_settings[0], ARRAY_SIZE(set_cmd1_settings),0x00FE);  // 

   // SET MADCTR

   // Step 14
   rm67162_write_cmd1(p, RM67162_CMD_SLPOUT, 0x0, 1);

   // step 15
   if (p->desc->delay.enable) {
     dev_err(panel->dev, "display enable time: %d\n",p->desc->delay.enable);
     msleep(120); // msleep(p->desc->delay.enable);
   }

   //  msleep(300);
   /* Addtional Steps for Step 13.
         Following two addtional steps are within step 13 from reading of table in specification document
   */

   /* Step 16 Display on  */
   rm67162_write_cmd1(p, RM67162_CMD_DISPON, 0,1);  //

   return 0;
}


static int panel_rm67162_disable(struct drm_panel *panel)
{
    struct panel_rm67162 *p = panel_to_ts(panel);

#ifdef TRACE
    printk("RM67162 -- panel_rm67162_disable() ");
#endif // TRACE

    if (!p->enabled)
        return 0;

    /* Step 0 set value to reset values for "sudo reboot"  */

    rm67162_sw_reset(p);
    /* Step 1 Display off  */
    rpi_short_write(p, RM67162_CMD_DISPOFF, 0,1);  //

    if (p->desc->delay.disable)
        msleep(p->desc->delay.disable);

    if (p->enable_gpio) {
        dev_err(panel->dev, "Disable display gpio\n");
        gpiod_set_value_cansleep(p->enable_gpio, 0);
    }

    if (p->desc->delay.disable)
        msleep(p->desc->delay.disable);

    /* Step 2 SLEEP IN */
//    rpi_short_write(p, RM67162_CMD_SLPIN, 0x0, 1);

    if (p->backlight) {
        p->backlight->props.power = FB_BLANK_POWERDOWN;
        p->backlight->props.state |= BL_CORE_FBBLANK;
        backlight_update_status(p->backlight);
    }


    p->enabled = false;

#ifdef TRACE
    printk("RM67162 -- panel_rm67162_disable(): done");
#endif // TRACE

    return 0;
}

static int panel_rm67162_unprepare(struct drm_panel *panel)
{
    struct panel_rm67162 *p = panel_to_ts(panel);

    failedCount = 0;
#ifdef TRACE
    printk("RM67162 -- panel_rm67162_unprepare() ");
#endif // TRACE

    if (!p->prepared)
        return 0;

    // Delay whilst the SLEEP commands take effect are handled in disable fn()


    if (p->enable_gpio)
        gpiod_set_value_cansleep(p->enable_gpio, 0);

    if (p->reset_gpio) {
        dev_err(panel->dev, "asserting display reset\n");
        gpiod_set_value_cansleep(p->reset_gpio, 0);
    }

    if (p->desc->delay.unprepare)
        msleep(p->desc->delay.unprepare);

    //regulator_disable(p->supply);
    regulator_bulk_disable(ARRAY_SIZE(p->supplies), p->supplies);

    p->prepared = false;

#ifdef TRACE
    printk("RM67162 -- panel_rm67162_unprepare(): done\n");
#endif // TRACE

    return 0;
}

/* Prepare 
*
*/
static int panel_rm67162_prepare(struct drm_panel *panel)
{
    struct panel_rm67162 *p = panel_to_ts(panel);
//    struct mipi_dsi_device *dsi = p->dsi;
    int err;

    failedCount = 0;
#ifdef TRACE
    printk("RM67162 -- panel_rm67162_prepare()\n");
#endif // TRACE

    if (p->prepared)
        return 0;

    // Make sure reglator is off
    if (p->enable_gpio) {
        dev_err(panel->dev, "Disable display gpio\n");
        gpiod_set_value_cansleep(p->enable_gpio, 0);
    }


    if (p->reset_gpio) {
        dev_err(panel->dev, "asserting display reset\n");
        gpiod_set_value_cansleep(p->reset_gpio, 0);
    }

    if (p->desc->delay.reset) {
        msleep(p->desc->delay.reset);    // reset assert time (min 10uS)
    }

    err = regulator_bulk_enable(ARRAY_SIZE(p->supplies), p->supplies);
    //err = regulator_enable(p->supply);
    if (err < 0) {
        dev_err(panel->dev, "failed to enable supply: %d\n", err);
        return err;
    }

    //await power to stabilise
    if (p->desc->delay.power_on) {
        dev_err(panel->dev, "display reset time: %d\n",p->desc->delay.power_on);
        msleep(p->desc->delay.power_on);
    }

    if (p->reset_gpio) {
        dev_err(panel->dev, "release display reset\n");
        gpiod_set_value_cansleep(p->reset_gpio, 1);
    }

    // post reset delay
    if (p->desc->delay.prepare) {
        msleep(p->desc->delay.prepare);
        dev_err(panel->dev, "post reset delay: %d\n",p->desc->delay.prepare);
    }

    err = rm67162_read_mtp_id(p);
    if(err != 0) {
        dev_err(panel->dev, "Failed to read MTP ID / MTP ID not supported: %d\n", err);
        return err;
    }

    err = writeManufacturerSettings(panel);
    if(err != 0 ) {
        dev_err(panel->dev, "failed to write manufacturer settings: %d, failedCount: %d Try again\n", err, failedCount);
	return err;
    }

    p->prepared = true;

#ifdef TRACE
    printk("RM67162 -- panel_rm67162_prepare(): done\n");
#endif //TRACE

    return 0;
}

static int panel_rm67162_enable(struct drm_panel *panel)
{
  struct panel_rm67162 *p = panel_to_ts(panel);
//  int err;


#ifdef TRACE
  printk("RM67162 -- panel_rm67162_enable()");
#endif // TRACE

  if (p->enabled)
  {
    return 0;
  }

/* KQ Do we need this
  if (p->desc->delay.enable) {
    dev_err(panel->dev, "display enable time: %d\n",p->desc->delay.enable);
    msleep(p->desc->delay.enable);
  }
*/


#if 0  // manufactory seeting move here as raspberry touchscreen did
      // read device id

    err = rm67162_read_mtp_id(p);
    if(err != 0) {
        dev_err(panel->dev, "Failed to read MTP ID / MTP ID not supported: %d\n", err);
        return err;
    }
#endif

#if 0
    err = rm67162_sw_reset(p);
    if(err != 0) {
        dev_err(panel->dev, "failed to reset panel : %d\n", err);
        return err;
    }
#endif


#if 0
    err = writeManufacturerSettings(panel);
    if(err != 0 ) {
        dev_err(panel->dev, "failed to write manufacturer settings: %d, failedCount: %d Try again\n", err, failedCount);
        return err;
    }
#endif

#if 0
    err = writeManufacturerSettings(panel);
    if(err != 0 ) {
        dev_err(panel->dev, "failed to write manufacturer settings: %d, failedCount: %d Try again\n", err, failedCount);
        failedCount = 0;
        err = rm67162_sw_reset(p);
        if(err != 0) {
          dev_err(panel->dev, "failed to reset panel : %d, really cannot do it, something wrong\n", err);
          return err;
        }
        msleep(10);
        err = writeManufacturerSettings(panel);
    }

    if(err != 0) {
        dev_err(panel->dev, "failed to write manufacturer settings: %d, failedCount: %d\n", err, failedCount);
        return err;
    }

    dev_info(panel->dev, "...... after writeManufacturerSettings: failedCount: %d\n",failedCount);
#endif

/*  KQ: Checking that the setup values were written to the panel (debug only)
    err = rm67162_check_values(panel);
    if(err != 0) {
        dev_err(panel->dev, "3 Failed to check some values %d\n", err);
    }
*/



  if (p->backlight) {
    p->backlight->props.state &= ~BL_CORE_FBBLANK;
    p->backlight->props.power = FB_BLANK_UNBLANK;
    backlight_update_status(p->backlight);
  }

  if (p->enable_gpio) {
    dev_err(panel->dev, "Enable display gpio\n");
    gpiod_set_value_cansleep(p->enable_gpio, 1);
  }

  p->enabled = true;

#ifdef TRACE
  printk("RM67162 -- panel_rm67162_enable():done\n");
#endif //TRACE

  return 0;
}

static int panel_rm67162_get_timings(struct drm_panel *panel,
                    unsigned int num_timings,
                    struct display_timing *timings)
{
    struct panel_rm67162 *p = panel_to_ts(panel);
    unsigned int i;

    if (p->desc->num_timings < num_timings)
        num_timings = p->desc->num_timings;

    if (timings)
        for (i = 0; i < num_timings; i++)
            timings[i] = p->desc->timings[i];

    return p->desc->num_timings;
}


static int panel_rm67162_get_modes(struct drm_panel *panel)
{
    struct drm_connector *connector = panel->connector;
    struct drm_device *drm = panel->drm;
    unsigned int i, num = 0;

    for (i = 0; i < ARRAY_SIZE(gov_rm67162_mode); i++) {
        const struct drm_display_mode *m = &gov_rm67162_mode[i];
        struct drm_display_mode *mode;

        mode = drm_mode_duplicate(drm, m);
        if (!mode) {
            dev_err(drm->dev, "failed to add mode %ux%u@%u\n",
                m->hdisplay, m->vdisplay, m->vrefresh);
            continue;
        }

        mode->type |= DRM_MODE_TYPE_DRIVER;

        if (i == 0)
            mode->type |= DRM_MODE_TYPE_PREFERRED;

        drm_mode_set_name(mode);

        drm_mode_probed_add(connector, mode);
        num++;
    }

    connector->display_info.bpc = 8;
    connector->display_info.width_mm = 30; //154;
    connector->display_info.height_mm = 30; //86;

    return num;
}

static const struct drm_panel_funcs panel_rm67162_funcs = {
    .disable = panel_rm67162_disable,
    .unprepare = panel_rm67162_unprepare,
    .prepare = panel_rm67162_prepare,
    .enable = panel_rm67162_enable,
    .get_timings = panel_rm67162_get_timings,
    .get_modes = panel_rm67162_get_modes,
};


static int panel_rm67162_probe(struct device *dev, const struct panel_desc *desc,
    struct mipi_dsi_device *dsi_dev)
{

/* struct device_node *endpoint, *dsi_host_node; */

/*        struct mipi_dsi_device_info info = {
                .type = DSI_DRIVER_NAME,
                .channel = 0,
                .node = NULL,
        };
*/

  struct device_node *backlight, *ddc;
  struct panel_rm67162 *panel;
  int err;

#ifdef TRACE
  printk("RM67162 -- panel_rm67162_probe()\n");
#endif //TRACE

  panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
  if (!panel)
    return -ENOMEM;

  mipi_dsi_set_drvdata(dsi_dev, panel);

  panel->enabled = false;
  panel->prepared = false;
  panel->desc = desc;
  panel->dsi = dsi_dev;

  //panel->supply = devm_regulator_get_optional(dev, "power");
  panel->supplies[0].supply = "power";
  panel->supplies[1].supply = "vss";

  err = devm_regulator_bulk_get(dev, ARRAY_SIZE(panel->supplies),
                      panel->supplies);
  if (err < 0) {
    dev_err(dev, "failed to get regulators: %d\n", err);
    return err;
  }

/*
  if (panel->supply) {
    if (IS_ERR(panel->supply)) {
      err = PTR_ERR(panel->supply);
      dev_err(dev, "failed to request power supply: %d\n", err);
      return err;
    }
  }
*/

  panel->reset_gpio = devm_gpiod_get_optional(dev, "reset",
                 GPIOD_OUT_LOW);

  if (panel->reset_gpio) {
    if (IS_ERR(panel->reset_gpio)) {
      err = PTR_ERR(panel->reset_gpio);
      dev_err(dev, "failed to request reset GPIO: %d\n", err);
      return err;
    }
    dev_err(dev, "setup reset GPIO: \n");
  }

  panel->enable_gpio = devm_gpiod_get_optional(dev, "enable",
                 GPIOD_OUT_LOW);

  if (panel->enable_gpio) {
    if (IS_ERR(panel->enable_gpio)) {
      err = PTR_ERR(panel->enable_gpio);
      dev_err(dev, "failed to request enable GPIO: %d\n", err);
      return err;
    }
    dev_err(dev, "setup enable  GPIO: \n");
  }


  backlight = of_parse_phandle(dev->of_node, "backlight", 0);
  if (backlight) {
    panel->backlight = of_find_backlight_by_node(backlight);
    of_node_put(backlight);
#ifdef TRACE
    printk("RM67162 -- panel_rm67162_probe() , panel backlight brightness %x\n", panel->backlight->props.brightness);
#endif
    if (!panel->backlight)
      return -EPROBE_DEFER;
  }

  ddc = of_parse_phandle(dev->of_node, "ddc-i2c-bus", 0);
  if (ddc) {
    panel->ddc = of_find_i2c_adapter_by_node(ddc);
    of_node_put(ddc);

    if (!panel->ddc) {
      err = -EPROBE_DEFER;
      goto free_backlight;
    }
  }


  drm_panel_init(&panel->base);
  panel->base.dev = dev;
  panel->base.funcs = &panel_rm67162_funcs;

  err = drm_panel_add(&panel->base);
  if (err < 0) {
      dev_err(dev, "probe: drm_panel_add failed: %d\n", err);
      goto free_ddc;
  }


#ifdef TRACE
  printk("RM67162 -- panel_rm67162_probe() success\n");
#endif //TRACE
  return 0;


free_ddc:
  if (panel->ddc)
    put_device(&panel->ddc->dev);
free_backlight:
  if (panel->backlight)
    put_device(&panel->backlight->dev);

  return err;
}

static int panel_rm67162_dsi_probe(struct mipi_dsi_device *dsi)
{
    const struct panel_desc_dsi *desc;
    const struct of_device_id *id;
    int err;

#ifdef TRACE
    printk("RM67162 - panel_rm67162_dsi_probe()\n");
#endif //TRACE

    id = of_match_node(dsi_of_match, dsi->dev.of_node);
    if (!id)
        return -ENODEV;

    desc = id->data;
    dsi->mode_flags = desc->flags;
    dsi->format = desc->format;
    dsi->lanes = desc->lanes;

    err = panel_rm67162_probe(&dsi->dev, &desc->desc, dsi);
    if (err < 0)
        return err;

#ifdef TRACE
    printk("RM67162 - panel_rm67162_dsi_probe() success\n");
#endif //TRACE

    return mipi_dsi_attach(dsi);
}

static int panel_rm67162_remove(struct device *dev)
{
    struct panel_rm67162 *panel = dev_get_drvdata(dev);

    drm_panel_remove(&panel->base);

    panel_rm67162_disable(&panel->base);
    panel_rm67162_unprepare(&panel->base);

    if (panel->ddc)
        put_device(&panel->ddc->dev);

    if (panel->backlight)
        put_device(&panel->backlight->dev);

    return 0;
}

static int panel_rm67162_dsi_remove(struct mipi_dsi_device *dsi)
{
    int err;

    err = mipi_dsi_detach(dsi);
    if (err < 0)
        dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", err);

    return panel_rm67162_remove(&dsi->dev);
}

static void panel_rm67162_shutdown(struct device *dev)
{
    struct panel_rm67162 *panel = dev_get_drvdata(dev);

    panel_rm67162_disable(&panel->base);
    panel_rm67162_unprepare(&panel->base);
}

static void panel_rm67162_dsi_shutdown(struct mipi_dsi_device *dsi)
{
    panel_rm67162_shutdown(&dsi->dev);
}

MODULE_DEVICE_TABLE(of, dsi_of_match);

static struct mipi_dsi_driver panel_rm67162_dsi_driver = {
    .driver = {
        .name = "panel-rm67162-dsi",
        .of_match_table = dsi_of_match,
    },
    .probe = panel_rm67162_dsi_probe,
    .remove = panel_rm67162_dsi_remove,
    .shutdown = panel_rm67162_dsi_shutdown,
};

module_mipi_dsi_driver(panel_rm67162_dsi_driver);

MODULE_AUTHOR("Kevin Quigley <keivn@kquigley.co.uk>");
MODULE_DESCRIPTION("Raspberry Pi GVO RM67162 Round OLED driver");
MODULE_LICENSE("GPL v2");
