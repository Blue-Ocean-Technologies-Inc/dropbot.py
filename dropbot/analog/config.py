# coding: utf-8
import pandas as pd

import arduino_helpers.hardware.teensy as teensy

CONVERSION_SPEEDS = dict(zip(['VERY_LOW_SPEED', 'LOW_SPEED', 'MED_SPEED',
                              'HI_SPEED', 'HI_SPEED_16_BITS',
                              'VERY_HIGH_SPEED'], range(6)))
SAMPLING_SPEEDS = dict(zip(['VERY_LOW_SPEED', 'LOW_SPEED', 'MED_SPEED',
                            'HIGH_SPEED', 'VERY_HIGH_SPEED'], range(5)))

ADC_SC2_REFSEL0_BIT = 0
ADC_CFG2_ADHSC = 0x04
# define ADC_CFG2_ADLSTS1_BIT (1)
# define ADC_CFG2_ADLSTS0_BIT (0)
# Sampling speed
ADC_CFG1_ADLSMP_BIT = 4
ADC_CFG2_ADLSTS1_BIT = 1
ADC_CFG2_ADLSTS0_BIT = 0

ADC_SC3_AVGE_BIT = 2
ADC_SC3_AVGS1_BIT = 1
ADC_SC3_AVGS0_BIT = 0


def ADC_CFG1_ADIV(n):
    return (n & 3) << 5  # Clock divide select, 0=direct, 1=div2, 2=div4, 3=div8


def ADC_CFG1_ADICLK(n):
    return (n & 3) << 0  # Input clock, 0=bus, 1=bus/2, 2=OSCERCLK, 3=async


# 36 MHz bus clock (see `proxy.D__F_BUS()`)
ADC_CFG1_2_25MHZ = (ADC_CFG1_ADIV(3) + ADC_CFG1_ADICLK(1))
ADC_CFG1_4_5MHZ = (ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1))
ADC_CFG1_9MHZ = (ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1))
ADC_CFG1_18MHZ = (ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(1))

ADC_CFG1_LOW_SPEED = ADC_CFG1_2_25MHZ
ADC_CFG1_VERY_LOW_SPEED = ADC_CFG1_LOW_SPEED
ADC_CFG1_MED_SPEED = ADC_CFG1_9MHZ
ADC_CFG1_HI_SPEED_16_BITS = ADC_CFG1_9MHZ
ADC_CFG1_HI_SPEED = ADC_CFG1_18MHZ
ADC_CFG1_VERY_HIGH_SPEED = ADC_CFG1_HI_SPEED


def _averaging(adc_config):
    if not (adc_config.SC3 & (1 << ADC_SC3_AVGE_BIT)):
        return 0
    else:
        avgs = adc_config.SC3 & ((1 << ADC_SC3_AVGS1_BIT) + (1 << ADC_SC3_AVGS0_BIT))
        return 2 ** (avgs + 2)


def _conversion_speed(adc_config):
    speed = adc_config.CFG1 & 0b1100011

    if adc_config.CFG2 & ADC_CFG2_ADHSC:
        # Either HI_SPEED_16_BITS, HI_SPEED, or VERY_HIGH_SPEED
        if speed == ADC_CFG1_VERY_HIGH_SPEED:
            return 'VERY_HIGH_SPEED'
        elif speed == ADC_CFG1_HI_SPEED:
            return 'HI_SPEED'
        else:  # if speed == ADC_CFG1_HI_SPEED_16_BITS:
            return 'HI_SPEED_16_BITS'
    else:
        # Either MED_SPEED, LOW_SPEED, or VERY_LOW_SPEED
        if speed == ADC_CFG1_MED_SPEED:
            return 'MED_SPEED'
        elif speed == ADC_CFG1_LOW_SPEED:
            return 'LOW_SPEED'
        else:  # if speed == ADC_CFG1_VERY_LOW_SPEED:
            return 'VERY_LOW_SPEED'


def _sampling_speed(adc_config):
    sampling_speed = 'VERY_LOW_SPEED'
    if not (adc_config.CFG1 & (1 << ADC_CFG1_ADLSMP_BIT)):
        sampling_speed = 'VERY_HIGH_SPEED'
    elif adc_config.CFG2 & ((1 << ADC_CFG2_ADLSTS1_BIT) | (1 << ADC_CFG2_ADLSTS0_BIT)) == (
            (1 << ADC_CFG2_ADLSTS1_BIT) | (1 << ADC_CFG2_ADLSTS0_BIT)):
        sampling_speed = 'HIGH_SPEED'
    elif adc_config.CFG2 & ((1 << ADC_CFG2_ADLSTS1_BIT) | (1 << ADC_CFG2_ADLSTS0_BIT)) == (
            (1 << ADC_CFG2_ADLSTS1_BIT) | (0 << ADC_CFG2_ADLSTS0_BIT)):
        sampling_speed = 'MED_SPEED'
    elif adc_config.CFG2 & ((1 << ADC_CFG2_ADLSTS1_BIT) | (1 << ADC_CFG2_ADLSTS0_BIT)) == (
            (0 << ADC_CFG2_ADLSTS1_BIT) | (1 << ADC_CFG2_ADLSTS0_BIT)):
        sampling_speed = 'LOW_SPEED'
    elif adc_config.CFG2 & ((1 << ADC_CFG2_ADLSTS1_BIT) | (1 << ADC_CFG2_ADLSTS0_BIT)) == (
            (0 << ADC_CFG2_ADLSTS1_BIT) | (0 << ADC_CFG2_ADLSTS0_BIT)):
        sampling_speed = 'VERY_LOW_SPEED'
    return sampling_speed


def _reference(adc_config):
    return 'ALT' if adc_config.SC2 & (1 << ADC_SC2_REFSEL0_BIT) else 'DEFAULT'


def _adc_config(proxy):
    adc_config_reg = pd.Series(proxy.analog_save_config(0), index=['SC1A', 'SC2', 'SC3', 'CFG1', 'CFG2'])
    adc_config = dict(zip(['reference', 'sampling_speed', 'conversion_speed', 'pga_gain', 'averaging'],
                          [_reference(adc_config_reg), _sampling_speed(adc_config_reg),
                           _conversion_speed(adc_config_reg),
                           proxy.getPGA(0) if proxy.isPGAEnabled(0) else 0,
                           _averaging(adc_config_reg)]))
    return adc_config


def _adc_load_config(proxy, adc_config, adc_num=0):
    """
    Apply configuration to ADC.

    Parameters
    ----------
    adc_config : dict
        ADC configuration including the following keys::
        - ``averaging``
        - ``conversion_speed``
        - ``sampling_speed``
        - ``pga_gain``
        - ``reference``
    adc_num : int, optional
        ADC index number (default=``0``).
    """
    proxy.setAveraging(adc_config['averaging'], adc_num)
    proxy.setConversionSpeed(CONVERSION_SPEEDS[adc_config['conversion_speed']],
                             adc_num)
    proxy.setSamplingSpeed(SAMPLING_SPEEDS[adc_config['sampling_speed']], adc_num)
    proxy.setReference(teensy.ADC_REF_3V3
                       if adc_config.get('reference') in ('DEFAULT', '3V3')
                       else teensy.ADC_REF_1V2, adc_num)
    if not adc_config['pga_gain']:
        proxy.disablePGA(adc_num)
    else:
        proxy.enablePGA(adc_config['pga_gain'], adc_num)
