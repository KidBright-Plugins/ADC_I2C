Blockly.JavaScript['adc_i2c.getRawValue'] = function(block) {
	return ['DEV_I2C1.adc_i2c(' + block.getFieldValue('CHANNEL') 
	+ ', ' 
	+ block.getFieldValue('ADDRESS') 
	+ ').getRawValue(' + block.getFieldValue('ANALOGINPUT')
	+ ')',
	Blockly.JavaScript.ORDER_ATOadc];
};

