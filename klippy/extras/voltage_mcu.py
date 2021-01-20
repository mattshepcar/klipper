#import logging
#import mcu

SAMPLE_TIME = 0.001
SAMPLE_COUNT = 8
REPORT_TIME = 0.300

class PrinterVoltageMCU:
    def __init__(self, config):
        self.printer = config.get_printer()
        # Read config
        mcu_name = config.get('mcu', 'mcu')
        # Setup ADC port
        ppins = config.get_printer().lookup_object('pins')
        self.mcu_adc = ppins.setup_pin('adc',
                                       '%s:ADC_VREF' % (mcu_name,))
        self.mcu_adc.setup_adc_callback(REPORT_TIME, self.adc_callback)
        self.mcu_adc.setup_minmax(SAMPLE_TIME, SAMPLE_COUNT)
        query_adc = config.get_printer().load_object(config, 'query_adc')
        query_adc.register_adc('vref:' + mcu_name, self.mcu_adc)

        self.vdd = 0
        self.readtime = 0

        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("VREF_MEASURE", "MCU", mcu_name,
                                   self.cmd_VREF_MEASURE,
                                   desc=self.cmd_VREF_MEASURE_help)

    def adc_callback(self, read_time, read_value):
        self.vdd = 1.20 / read_value
        self.readtime = read_time

    cmd_VREF_MEASURE_help = "Report MCU voltage"
    def cmd_VREF_MEASURE(self, gcmd):
        gcmd.respond_info("VREF = %.1fv" % self.vdd)

def load_config(config):
    return PrinterVoltageMCU(config)
