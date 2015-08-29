
function sensorData(id, time, temp, rh, press, light, pir) {
	this.id = id;
	this.time = time;
	this.temp = temp;
	this.rh = rh;
	this.press = press;
	this.light = light;
	this.pir = pir;
}

var sensors = new Array([]);


var addData = function(message) {
	if (message.id === 1 | message.id === 2 | message.id === 3) {
		if (typeof sensors[message.id] === 'undefined') {
			sensors[message.id] = new Array();
		}
		if (message.pir === 0 || message.pir === 1) {
			(sensors[message.id]).push(new sensorData(message.id, message.time, message.temp, message.rh, message.press, message.light, message.pir));	

		updateSensorData(message.id);
		}
		//processData(message.id);
	}
}

var updateSensorData = function(sensorId) {
	var latestTime = sensors[sensorId][sensors[sensorId].length - 1].time;

	sensors[sensorId] = sensors[sensorId].filter(function(data) {
		return (latestTime - data.time) < historyTime;
	});
}

var epochToDateTime = function(dateTime) {
	var date = new Date(dateTime);
	var dateString = (date.getMonth() + 1) + "-" + date.getDate() + "-" + date.getFullYear() + " " + date.getHours() + ":" + date.getMinutes() + ":" + date.getSeconds();
	return dateString;
}

var processData = function(sensorId) {
	var sum_temp = 0;
	var sumsqr_temp = 0;

	var sum_light = 0;
	var sumsqr_light = 0;

	sensors[sensorId].forEach( function(value) {
		sum_temp += value.temp;
		sumsqr_temp += value.temp * value.temp;
	
		sum_light += value.light;
		sumsqr_light += value.light * value.light;
	});


	var mean_temp = sum_temp / (sensors[sensorId].length);
	var varience_temp = (sumsqr_temp / sensors[sensorId].length) - (mean_temp * mean_temp);
	var stdev = Math.sqrt(varience_);

	sensors[sensorId] = sensors[sensorId].filter( function(d) {
		var bool = (d.temp >= mean - 3*stdev) && (d.temp <= mean + 3*stdev);
		//console.log(bool);
		//console.log (mean);
		//console.log(varience);
		return bool;
	});
	
}

//console.log(epochToDateTime(1438800980756));


