/*
 * bq2419x-charger.c -- BQ24190/BQ24192/BQ24192i/BQ24193 Charger
 *                     regulator driver
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/slab.h>

/* Register definitions */
#define BQ2419X_INPUT_SRC_REG                  0x00
#define BQ2419X_PWR_ON_REG                     0x01
#define BQ2419X_CHRG_CTRL_REG                  0x02
#define BQ2419X_CHRG_TERM_REG                  0x03
#define BQ2419X_VOLT_CTRL_REG                  0x04
#define BQ2419X_TIME_CTRL_REG                  0x05
#define BQ2419X_THERM_REG                      0x06
#define BQ2419X_MISC_OPER_REG                  0x07
#define BQ2419X_SYS_STAT_REG                   0x08
#define BQ2419X_FAULT_REG                      0x09
#define BQ2419X_REVISION_REG                   0x0a
#define BQ2419X_MAX_REGS                       (BQ2419X_REVISION_REG + 1)

/* REG00 INPUT_SRC_REG */
#define BQ2419X_INPUT_SRC_EN_HIZ_MASK          BIT(7)
#define BQ2419X_INPUT_SRC_VINDPM_MASK          0x78
#define BQ2419X_INPUT_SRC_VINDPM_SHIFT         3
#define BQ2419X_INPUT_SRC_IINLIM_MASK          0x7

/* REG01 PWR_ON_REG */
#define BQ2419X_PWR_ON_OTG_MASK                        BIT(5)
#define BQ2419X_PWR_ON_OTG_ENABLE              BIT(5)
#define BQ2419X_PWR_ON_CHARGER_MASK            BIT(4)
#define BQ2419X_PWR_ON_CHARGER_ENABLE          BIT(4)

/* REG0A REVISION_REG */
#define BQ24190_IC_VER                         0x40
#define BQ24192_IC_VER                         0x28
#define BQ24192i_IC_VER                                0x18

/* REG05 TIME_CTRL_REG */
#define BQ2419X_TIME_CTRL_WD_MASK              0x30
#define BQ2419X_TIME_CTRL_WD_DISABLE           0x00
#define BQ2419X_TIME_CTRL_WD_40ms              0x10
#define BQ2419X_TIME_CTRL_WD_80ms              0x20
#define BQ2419X_TIME_CTRL_WD_160ms             0x30
#define BQ2419X_TIME_CTRL_EN_SFT_TIMER_MASK    BIT(3)

/* REG08 SYS_STAT_REG */
#define BQ2419x_SYS_STAT_CHRG_STATE_MASK               0x30
#define BQ2419x_SYS_STAT_CHRG_STATE_NOTCHARGING                0x00
#define BQ2419x_SYS_STAT_CHRG_STATE_PRE_CHARGE         0x10
#define BQ2419x_SYS_STAT_CHRG_STATE_POST_CHARGE                0x20
#define BQ2419x_SYS_STAT_CHRG_STATE_CHARGE_DONE                0x30

/* REG09 BQ2419X_FAULT_REG */
#define BQ2419x_FAULT_WATCHDOG_FAULT           BIT(7)
#define BQ2419x_FAULT_BOOST_FAULT              BIT(6)
#define BQ2419x_FAULT_CHRG_FAULT_MASK          0x30
#define BQ2419x_FAULT_CHRG_NORMAL              0x00
#define BQ2419x_FAULT_CHRG_INPUT               0x10
#define BQ2419x_FAULT_CHRG_THERMAL             0x20
#define BQ2419x_FAULT_CHRG_SAFTY               0x30
#define BQ2419x_FAULT_NTC_FAULT                        0x07

/* Input current limit */
static const unsigned int bq2419x_charging_current[] = {
       100, 150, 500, 900, 1200, 1500, 2000, 3000,
};

enum regulator_id {
       BQ2419X_REGULATOR_VBUS,
       BQ2419X_REGULATOR_CHARGER,
};

struct bq2419x_chip {
       struct device                   *dev;
       struct regmap                   *regmap;
       int                             irq;

       struct mutex                    mutex;
       int                             otg_iusb_gpio;
       int                             fast_charge_current_limit;
       int                             max_in_voltage_limit;
       int                             wdt_timeout;
       int                             wdt_refresh_timeout;
       struct delayed_work             bq_wdt_work;

       struct regulator_dev            *chg_rdev;
       struct regulator_dev            *vbus_rdev;
};

static int bq2419x_set_charging_current(struct regulator_dev *rdev,
               int min_uA, int max_uA);

static struct regulator_ops bq2419x_vbus_reg_ops = {
       .enable         = regulator_enable_regmap,
       .disable        = regulator_disable_regmap,
       .is_enabled     = regulator_is_enabled_regmap,
};

static struct regulator_ops bq2419x_charger_regulator_ops = {
       .set_current_limit = bq2419x_set_charging_current,
};

static struct regulator_desc bq2419x_reg_desc[] =  {
       {
               .name = "bq2419x-vbus",
               .owner = THIS_MODULE,
               .type = REGULATOR_VOLTAGE,
               .ops = &bq2419x_vbus_reg_ops,
               .enable_mask = BQ2419X_PWR_ON_OTG_MASK,
               .enable_reg = BQ2419X_PWR_ON_REG,
               .enable_time = 500000,
       }, {
               .name  = "bq2419x-charger",
               .owner = THIS_MODULE,
               .type  = REGULATOR_CURRENT,
               .ops = &bq2419x_charger_regulator_ops,
       },
};

static struct of_regulator_match bq2419x_matches[] = {
       { .name = "vbus", },
       { .name = "charger", },
};

static int bq2419x_parse_dt_reg_data(struct bq2419x_chip *bq2419x)
{
       struct device_node *np = of_node_get(bq2419x->dev->of_node);
       struct device_node *regulators;
       int ret;
       u32 prop;

       regulators = of_find_node_by_name(np, "regulators");
       if (!regulators) {
               dev_err(bq2419x->dev, "regulator node not found\n");
               return -ENODEV;
       }

       ret = of_regulator_match(bq2419x->dev, regulators, bq2419x_matches,
                               ARRAY_SIZE(bq2419x_matches));
       of_node_put(regulators);
       if (ret < 0) {
               dev_err(bq2419x->dev,
                       "Parsing of regulator init data failed %d\n", ret);
               return ret;
       }

       ret = of_property_read_u32(np, "ti,watchdog-timeout", &prop);
       if (!ret)
               bq2419x->wdt_timeout = prop;

       ret = of_property_read_u32(np, "ti,maximum-in-voltage-limit", &prop);
       if (!ret)
               bq2419x->max_in_voltage_limit = prop;

       ret = of_property_read_u32(np, "ti,fast-charge-current-limit", &prop);
       if (!ret)
               bq2419x->fast_charge_current_limit = prop;

       bq2419x->otg_iusb_gpio = of_get_named_gpio(np, "otg-iusb-gpio", 0);
       if ((bq2419x->otg_iusb_gpio == -ENODEV) ||
               (bq2419x->otg_iusb_gpio == -EPROBE_DEFER))
               return -EPROBE_DEFER;

       return 0;
}

static int bq2419x_clear_hi_z(struct bq2419x_chip *bq2419x)
{
       int ret;

       ret = regmap_update_bits(bq2419x->regmap, BQ2419X_INPUT_SRC_REG,
                       BQ2419X_INPUT_SRC_EN_HIZ_MASK, 0);
       if (ret < 0) {
               dev_err(bq2419x->dev, "INPUT_SRC_REG update failed %d\n", ret);
               return ret;
       }
       return ret;
}

static int bq2419x_configure_charging_current(struct bq2419x_chip *bq2419x,
               int in_current)
{
       int ret;
       int i;

       bq2419x_clear_hi_z(bq2419x);
       for (i = 0; i < ARRAY_SIZE(bq2419x_charging_current) - 1; ++i) {
               if (in_current <= bq2419x_charging_current[i])
                       break;
       }
       ret = regmap_update_bits(bq2419x->regmap, BQ2419X_INPUT_SRC_REG,
                               BQ2419X_INPUT_SRC_IINLIM_MASK, i);
       if (ret < 0)
               dev_err(bq2419x->dev, "INPUT_SRC_REG update failed %d\n", ret);
       return ret;
}

static int bq2419x_set_charging_enable(struct bq2419x_chip *bq2419x,
               bool enable)
{
       int ret;
       int val = 0;

       if (enable)
               val = BQ2419X_PWR_ON_CHARGER_ENABLE;

       ret = regmap_update_bits(bq2419x->regmap, BQ2419X_PWR_ON_REG,
                        BQ2419X_PWR_ON_CHARGER_MASK, val);
       if (ret < 0) {
               dev_err(bq2419x->dev, "PWR_ON_REG update failed %d\n", ret);
               return ret;
       }
       return ret;
}

static int bq2419x_charger_init_configure(struct bq2419x_chip *bq2419x)
{
       int ret;
       int fast_charging_current;
       int in_voltage_limit;

       /* Configure fast charging current */
       fast_charging_current = bq2419x->fast_charge_current_limit;
       if (fast_charging_current < 512)
               fast_charging_current = 512;
       fast_charging_current = (fast_charging_current - 512) / 64;
       ret = regmap_write(bq2419x->regmap, BQ2419X_CHRG_CTRL_REG,
                               fast_charging_current << 2);
       if (ret < 0) {
               dev_err(bq2419x->dev, "CHRG_CTRL_REG write failed %d\n", ret);
               return ret;
       }

       /* Configure input voltage limit */
       in_voltage_limit = bq2419x->max_in_voltage_limit;
       if (in_voltage_limit < 3880)
               in_voltage_limit = 3880;
       in_voltage_limit -= 3880;
       in_voltage_limit = (in_voltage_limit/80) <<
                               BQ2419X_INPUT_SRC_VINDPM_SHIFT;
       ret = regmap_update_bits(bq2419x->regmap, BQ2419X_INPUT_SRC_REG,
                       BQ2419X_INPUT_SRC_VINDPM_MASK, in_voltage_limit);
       if (ret < 0) {
               dev_err(bq2419x->dev, "INPUT_SRC_REG update failed %d\n", ret);
               return ret;
       }

       ret = bq2419x_clear_hi_z(bq2419x);
       if (ret < 0) {
               dev_err(bq2419x->dev, "Clearing HiZ failed %d\n", ret);
               return ret;
       }

       if (bq2419x_matches[BQ2419X_REGULATOR_CHARGER].init_data) {
               ret = bq2419x_set_charging_enable(bq2419x, true);
               if (ret < 0) {
                       dev_err(bq2419x->dev, "Charger enable failed %d\n",
                               ret);
                       return ret;
               }
       }
       return ret;
}

static int bq2419x_set_charging_current(struct regulator_dev *rdev,
                                       int min_uA, int max_uA)
{
       struct bq2419x_chip *bq2419x = rdev_get_drvdata(rdev);
       int ret = 0;
       int val;
       int in_current_limit = max_uA/1000;

       ret = regmap_read(bq2419x->regmap, BQ2419X_SYS_STAT_REG, &val);
       if (ret < 0) {
               dev_err(bq2419x->dev, "SYS_STAT_REG read failed %d\n", ret);
               return ret;
       }

       if (max_uA == 0 && val != 0)
               return ret;

       ret = bq2419x_configure_charging_current(bq2419x, in_current_limit);
       if (ret < 0) {
               dev_err(bq2419x->dev, "Charger enable failed %d\n", ret);
               return ret;
       }

       ret = bq2419x_set_charging_enable(bq2419x, true);
       if (ret < 0) {
               dev_err(bq2419x->dev, "Charger enable failed %d\n", ret);
               return ret;
       }

       ret = bq2419x_clear_hi_z(bq2419x);
       if (ret < 0) {
               dev_err(bq2419x->dev, "Clearing HiZ failed %d\n", ret);
               return ret;
       }
       return ret;
}

static int bq2419x_reset_wdt(struct bq2419x_chip *bq2419x)
{
       int ret = 0;
       unsigned int reg01;

       mutex_lock(&bq2419x->mutex);

       ret = bq2419x_clear_hi_z(bq2419x);
       if (ret < 0) {
               dev_err(bq2419x->dev, "Clearing HiZ failed %d\n", ret);
               return ret;
       }

       ret = regmap_read(bq2419x->regmap, BQ2419X_PWR_ON_REG, &reg01);
       if (ret < 0) {
               dev_err(bq2419x->dev, "PWR_ON_REG read failed %d\n", ret);
               goto scrub;
       }

       reg01 |= BIT(6);

       /* Write two times to make sure reset WDT as suggested by Vendor*/
       ret = regmap_write(bq2419x->regmap, BQ2419X_PWR_ON_REG, reg01);
       if (ret < 0) {
               dev_err(bq2419x->dev, "PWR_ON_REG write failed %d\n", ret);
               goto scrub;
       }
       ret = regmap_write(bq2419x->regmap, BQ2419X_PWR_ON_REG, reg01);
       if (ret < 0) {
               dev_err(bq2419x->dev, "PWR_ON_REG write failed %d\n", ret);
               goto scrub;
       }

scrub:
       mutex_unlock(&bq2419x->mutex);
       return ret;
}

static int bq2419x_fault_clear_sts(struct bq2419x_chip *bq2419x)
{
       int ret;
       unsigned int reg09;

       /* Read two times for clearing the FAULT REG */
       ret = regmap_read(bq2419x->regmap, BQ2419X_FAULT_REG, &reg09);
       if (ret < 0) {
               dev_err(bq2419x->dev, "FAULT_REG read failed %d\n", ret);
               return ret;
       }

       ret = regmap_read(bq2419x->regmap, BQ2419X_FAULT_REG, &reg09);
       if (ret < 0)
               dev_err(bq2419x->dev, "FAULT_REG read failed %d\n", ret);

       return ret;
}

static int bq2419x_watchdog_init(struct bq2419x_chip *bq2419x, int timeout)
{
       int ret, val;
       unsigned int reg05;

       if (!timeout) {
               ret = regmap_update_bits(bq2419x->regmap, BQ2419X_TIME_CTRL_REG,
                               BQ2419X_TIME_CTRL_WD_MASK, 0);
               if (ret < 0)
                       dev_err(bq2419x->dev,
                               "TIME_CTRL_REG read failed %d\n", ret);
               return ret;
       }

       if (timeout <= 60) {
               val = BQ2419X_TIME_CTRL_WD_40ms;
               bq2419x->wdt_refresh_timeout = 25;
       } else if (timeout <= 120) {
               val = BQ2419X_TIME_CTRL_WD_80ms;
               bq2419x->wdt_refresh_timeout = 50;
       } else {
               val = BQ2419X_TIME_CTRL_WD_160ms;
               bq2419x->wdt_refresh_timeout = 125;
       }

       ret = regmap_read(bq2419x->regmap, BQ2419X_TIME_CTRL_REG, &reg05);
       if (ret < 0) {
               dev_err(bq2419x->dev, "TIME_CTRL_REG read failed %d\n", ret);
               return ret;
       }

       if ((reg05 & BQ2419X_TIME_CTRL_WD_MASK) != val) {
               ret = regmap_update_bits(bq2419x->regmap, BQ2419X_TIME_CTRL_REG,
                               BQ2419X_TIME_CTRL_WD_MASK, val);
               if (ret < 0) {
                       dev_err(bq2419x->dev,
                               "TIME_CTRL_REG read failed %d\n", ret);
                       return ret;
               }
       }

       ret = bq2419x_reset_wdt(bq2419x);
       if (ret < 0)
               dev_err(bq2419x->dev, "bq2419x_reset_wdt failed %d\n", ret);

       return ret;
}

static void bq2419x_reset_wdt_work(struct work_struct *work)
{
       struct bq2419x_chip *bq2419x;
       int ret;

       bq2419x = container_of(work, struct bq2419x_chip, bq_wdt_work.work);
       ret = bq2419x_reset_wdt(bq2419x);
       if (ret < 0)
               dev_err(bq2419x->dev, "bq2419x_reset_wdt failed %d\n", ret);
       schedule_delayed_work(&bq2419x->bq_wdt_work,
                       msecs_to_jiffies(bq2419x->wdt_refresh_timeout * 1000));
}

static int bq2419x_reset_safety_timer(struct bq2419x_chip *bq2419x)
{
       int ret;

       ret = regmap_update_bits(bq2419x->regmap, BQ2419X_TIME_CTRL_REG,
                       BQ2419X_TIME_CTRL_EN_SFT_TIMER_MASK, 0);
       if (ret < 0) {
               dev_err(bq2419x->dev,
                               "TIME_CTRL_REG update failed %d\n", ret);
               return ret;
       }

       ret = regmap_update_bits(bq2419x->regmap, BQ2419X_TIME_CTRL_REG,
                       BQ2419X_TIME_CTRL_EN_SFT_TIMER_MASK,
                       BQ2419X_TIME_CTRL_EN_SFT_TIMER_MASK);
       if (ret < 0)
               dev_err(bq2419x->dev,
                               "TIME_CTRL_REG update failed %d\n", ret);
       return ret;
}

static irqreturn_t bq2419x_irq(int irq, void *data)
{
       struct bq2419x_chip *bq2419x = data;
       int ret;
       unsigned int val;
       int check_chg_state = 0;

       ret = regmap_read(bq2419x->regmap, BQ2419X_FAULT_REG, &val);
       if (ret < 0) {
               dev_err(bq2419x->dev, "FAULT_REG read failed %d\n", ret);
               return ret;
       }

       if (val & BQ2419x_FAULT_WATCHDOG_FAULT) {
               dev_err(bq2419x->dev,
                       "Charging Fault: Watchdog Timer Expired\n");

               ret = bq2419x_watchdog_init(bq2419x, bq2419x->wdt_timeout);
               if (ret < 0) {
                       dev_err(bq2419x->dev, "BQWDT init failed %d\n", ret);
                       return ret;
               }

               ret = bq2419x_charger_init_configure(bq2419x);
               if (ret < 0) {
                       dev_err(bq2419x->dev, "Charger init failed %d\n", ret);
                       return ret;
               }
       }

       if (val & BQ2419x_FAULT_BOOST_FAULT)
               dev_err(bq2419x->dev, "Charging Fault: VBUS Overloaded\n");

       switch (val & BQ2419x_FAULT_CHRG_FAULT_MASK) {
       case BQ2419x_FAULT_CHRG_INPUT:
               dev_err(bq2419x->dev,
                       "Charging Fault: VBUS OVP or VBAT<VBUS<3.8V\n");
               break;
       case BQ2419x_FAULT_CHRG_THERMAL:
               dev_err(bq2419x->dev, "Charging Fault: Thermal shutdown\n");
               check_chg_state = 1;
               break;
       case BQ2419x_FAULT_CHRG_SAFTY:
               dev_err(bq2419x->dev,
                       "Charging Fault: Safety timer expiration\n");
               ret = bq2419x_reset_safety_timer(bq2419x);
               if (ret < 0) {
                       dev_err(bq2419x->dev, "Reset safety timer failed %d\n",
                                                       ret);
                       return ret;
               }
               break;
       default:
               break;
       }

       if (val & BQ2419x_FAULT_NTC_FAULT)
               dev_err(bq2419x->dev, "Charging Fault: NTC fault %d\n",
                               val & BQ2419x_FAULT_NTC_FAULT);

       ret = bq2419x_fault_clear_sts(bq2419x);
       if (ret < 0) {
               dev_err(bq2419x->dev, "fault clear status failed %d\n", ret);
               return ret;
       }

       ret = regmap_read(bq2419x->regmap, BQ2419X_SYS_STAT_REG, &val);
       if (ret < 0) {
               dev_err(bq2419x->dev, "SYS_STAT_REG read failed %d\n", ret);
               return ret;
       }

       if ((val & BQ2419x_SYS_STAT_CHRG_STATE_MASK) ==
                               BQ2419x_SYS_STAT_CHRG_STATE_CHARGE_DONE)
               dev_info(bq2419x->dev, "Charging completed\n");

       return IRQ_HANDLED;
}

static int bq2419x_init_charger_regulator(struct bq2419x_chip *bq2419x)
{
       int ret = 0;
       struct regulator_config rconfig = { };

       rconfig.dev = bq2419x->dev;
       rconfig.of_node = bq2419x->dev->of_node;
       rconfig.init_data =
                       bq2419x_matches[BQ2419X_REGULATOR_CHARGER].init_data;
       rconfig.driver_data = bq2419x;
       bq2419x->chg_rdev = regulator_register(
                               &bq2419x_reg_desc[BQ2419X_REGULATOR_CHARGER],
                               &rconfig);
       if (IS_ERR(bq2419x->chg_rdev)) {
               ret = PTR_ERR(bq2419x->chg_rdev);
               dev_err(bq2419x->dev, "charger regulator register failed %d\n",
                       ret);
       }
       return ret;
}

static int bq2419x_init_vbus_regulator(struct bq2419x_chip *bq2419x)
{
       int ret = 0;
       struct regulator_config rconfig = { };

       if (gpio_is_valid(bq2419x->otg_iusb_gpio)) {
               ret = devm_gpio_request_one(bq2419x->dev,
                               bq2419x->otg_iusb_gpio,
                               GPIOF_OUT_INIT_HIGH,
                               dev_name(bq2419x->dev));
               if (ret < 0) {
                       dev_err(bq2419x->dev, "gpio request failed %d\n", ret);
                       return ret;
               }
       }

       /* Register the regulators */
       rconfig.dev = bq2419x->dev;
       rconfig.of_node = bq2419x->dev->of_node;
       rconfig.init_data = bq2419x_matches[BQ2419X_REGULATOR_VBUS].init_data;
       rconfig.driver_data = bq2419x;
       bq2419x->vbus_rdev = regulator_register(
                               &bq2419x_reg_desc[BQ2419X_REGULATOR_VBUS],
                               &rconfig);
       if (IS_ERR(bq2419x->vbus_rdev)) {
               ret = PTR_ERR(bq2419x->vbus_rdev);
               dev_err(bq2419x->dev, "VBUS regulator register failed %d\n",
                       ret);
               return ret;
       }
       return ret;
}

static int bq2419x_show_chip_version(struct bq2419x_chip *bq2419x)
{
       int ret;
       unsigned int val;

       ret = regmap_read(bq2419x->regmap, BQ2419X_REVISION_REG, &val);
       if (ret < 0) {
               dev_err(bq2419x->dev, "REVISION_REG read failed %d\n", ret);
               return ret;
       }

       if ((val & BQ24190_IC_VER) == BQ24190_IC_VER)
               dev_info(bq2419x->dev, "chip type BQ24190 detected\n");
       else if ((val & BQ24192_IC_VER) == BQ24192_IC_VER)
               dev_info(bq2419x->dev, "chip type BQ2419X/3 detected\n");
       else if ((val & BQ24192i_IC_VER) == BQ24192i_IC_VER)
               dev_info(bq2419x->dev, "chip type BQ2419Xi detected\n");
       return 0;
}


static const struct regmap_config bq2419x_regmap_config = {
       .reg_bits               = 8,
       .val_bits               = 8,
       .max_register           = BQ2419X_MAX_REGS,
};

static int bq2419x_probe(struct i2c_client *client,
                               const struct i2c_device_id *id)
{
       struct bq2419x_chip *bq2419x;
       int ret = 0;

       if (!client->dev.of_node) {
               dev_err(&client->dev, "Driver only supported from DT\n");
               return -ENODEV;
       }


       bq2419x = devm_kzalloc(&client->dev, sizeof(*bq2419x), GFP_KERNEL);
       if (!bq2419x) {
               dev_err(&client->dev, "Memory allocation failed\n");
               return -ENOMEM;
       }

       bq2419x->dev = &client->dev;
       bq2419x->irq = client->irq;

       ret = bq2419x_parse_dt_reg_data(bq2419x);
       if (ret < 0) {
               dev_err(&client->dev, "DT parsing failed %d\n", ret);
               return ret;
       }

       bq2419x->regmap = devm_regmap_init_i2c(client, &bq2419x_regmap_config);
       if (IS_ERR(bq2419x->regmap)) {
               ret = PTR_ERR(bq2419x->regmap);
               dev_err(&client->dev, "regmap init failed %d\n", ret);
               return ret;
       }


       i2c_set_clientdata(client, bq2419x);
       mutex_init(&bq2419x->mutex);

       ret = bq2419x_show_chip_version(bq2419x);
       if (ret < 0) {
               dev_err(&client->dev, "version read failed %d\n", ret);
               goto scrub_mutex;
       }

       ret = bq2419x_charger_init_configure(bq2419x);
       if (ret < 0) {
               dev_err(bq2419x->dev, "Charger init failed %d\n", ret);
               goto scrub_mutex;
       }

       ret = bq2419x_watchdog_init(bq2419x, bq2419x->wdt_timeout);
       if (ret < 0) {
               dev_err(bq2419x->dev, "BQWDT init failed %d\n", ret);
               goto scrub_mutex;
       }

       ret = bq2419x_init_vbus_regulator(bq2419x);
       if (ret < 0) {
               dev_err(&client->dev, "VBUS regualtor init failed %d\n", ret);
               goto scrub_mutex;
       }

       ret = bq2419x_init_charger_regulator(bq2419x);
       if (ret < 0) {
               dev_err(&client->dev, "Charger regualtor init failed %d\n",
                       ret);
               goto scrub_vbus_reg;
       }

       ret = bq2419x_fault_clear_sts(bq2419x);
       if (ret < 0) {
               dev_err(bq2419x->dev, "fault clear status failed %d\n", ret);
               goto scrub_chg_reg;
       }

       if (bq2419x->wdt_timeout) {
               INIT_DELAYED_WORK(&bq2419x->bq_wdt_work,
                                               bq2419x_reset_wdt_work);
               schedule_delayed_work(&bq2419x->bq_wdt_work,
                       msecs_to_jiffies(bq2419x->wdt_refresh_timeout * 1000));
       }

       ret = devm_request_threaded_irq(bq2419x->dev, bq2419x->irq, NULL,
               bq2419x_irq, IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
                       dev_name(bq2419x->dev), bq2419x);
       if (ret < 0) {
               dev_err(bq2419x->dev, "request IRQ %d failed %d\n",
                               bq2419x->irq, ret);
               goto scrub_wq;
       }

       return 0;

scrub_wq:
       flush_work(&bq2419x->bq_wdt_work.work);
scrub_chg_reg:
       regulator_unregister(bq2419x->chg_rdev);
scrub_vbus_reg:
       regulator_unregister(bq2419x->vbus_rdev);
scrub_mutex:
       mutex_destroy(&bq2419x->mutex);
       return ret;
}

static int bq2419x_remove(struct i2c_client *client)
{
       struct bq2419x_chip *bq2419x = i2c_get_clientdata(client);

       flush_work(&bq2419x->bq_wdt_work.work);
       regulator_unregister(bq2419x->vbus_rdev);
       regulator_unregister(bq2419x->chg_rdev);
       mutex_destroy(&bq2419x->mutex);
       return 0;
}

static const struct of_device_id bq2419x_of_match[] = {
        { .compatible = "ti,bq2419x",},
        {},
};
MODULE_DEVICE_TABLE(of, tps51632_of_match);

static const struct i2c_device_id bq2419x_id[] = {
       {.name = "bq2419x",},
       {},
};
MODULE_DEVICE_TABLE(i2c, bq2419x_id);

static struct i2c_driver bq2419x_i2c_driver = {
       .driver = {
               .name   = "bq2419x",
               .owner  = THIS_MODULE,
               .of_match_table = of_match_ptr(bq2419x_of_match),
       },
       .probe          = bq2419x_probe,
       .remove         = bq2419x_remove,
       .id_table       = bq2419x_id,
};

static int __init bq2419x_module_init(void)
{
       return i2c_add_driver(&bq2419x_i2c_driver);
}
subsys_initcall(bq2419x_module_init);

static void __exit bq2419x_cleanup(void)
{
       i2c_del_driver(&bq2419x_i2c_driver);
}
module_exit(bq2419x_cleanup);

MODULE_DESCRIPTION("BQ24190/BQ24192/BQ24192i/BQ24193 battery charger driver");
MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_LICENSE("GPL v2");
