Blockly.Blocks["adc_i2c.getRawValue"] = {
	init: function() {
		this.appendDummyInput()
			.appendField(Blockly.Msg.GET_ADC_I2C_TITLE);

		var channel_array = [];
		for (var i = 0;i <= 0; i++) {
			channel_array.push([String(i), String(i)]);
		}
		this.appendDummyInput()
			.appendField(Blockly.Msg.CHANNEL)
			.appendField(new Blockly.FieldDropdown(channel_array), 'CHANNEL');

		// device address
		this.appendDummyInput()
			.appendField(Blockly.Msg.ADDRESS)
			.appendField(new Blockly.FieldDropdown([
				["0x48", "72"]
			]), 'ADDRESS');
			
		this.appendDummyInput()
			.appendField("Analog Input")
			.appendField(new Blockly.FieldDropdown([
				["A0", "0"],
				["A1", "1"],
				["A2", "2"],
				["A3", "3"]
			]), 'ANALOGINPUT');

		this.setOutput(true, 'Number');
		this.setPreviousStatement(false);
		this.setNextStatement(false);
		this.setColour(58);
		this.setTooltip(Blockly.Msg.GET_ADC_I2C_TOOLTIP);
		this.setHelpUrl(Blockly.Msg.GET_ADC_I2C_HELPURL);
	}
};
