// an array of arrays
// first index is sensor id, second is the data objects
var sensors = new Array();

// new sensordata object
/*function sensorData(id, temp, rh, press, light, pir, xmag, ymag, zmag, xaccel, yaccel, zaccel, time) {
	this.id = id;
	this.temp = temp;
	this.rh = rh;
	this.press = press;
	this.pir = pir;
	this.xmag = xmag;
	this.ymag = ymag;
	this.zmag = zmag;
	this.xaccel = xaccel;
	this.yaccel = yaccel;
	this.zaccel = zaccel;
	this.time = time;
}*/

function sensorData(id) {
	this.id = id;
	this.temp = [];
	this.rh = [];
	this.press = [];
	this.light = [];
	this.pir =  [];
	this.xmag = [];
	this.ymag = [];
	this.zmag = [];
	this.xaccel = [];
	this.yaccel = [];
	this.zaccel =[];
	this.accel = [];
}

sensors[1] = new sensorData(1);

// removes old data points according current time - historytime
var updateSensorData = function(sensorID, historyTime) {
	var curTime = (new Date()).getTime();

	sensors[sensorID].temp = sensors[sensorID].temp.filter(function(data) {
			return (curTime - (data.x).getTime()) < historyTime;
		});
	sensors[sensorID].rh = sensors[sensorID].rh.filter(function(data) {
			return (curTime - (data.x).getTime()) < historyTime;
		});
	sensors[sensorID].press = sensors[sensorID].press.filter(function(data) {
			return (curTime - (data.x).getTime()) < historyTime;
		});
	sensors[sensorID].light = sensors[sensorID].light.filter(function(data) {
			return (curTime - (data.x).getTime()) < historyTime;
		});
	sensors[sensorID].pir = sensors[sensorID].pir.filter(function(data) {
			return (curTime - (data.x).getTime()) < historyTime;
		});
	sensors[sensorID].xmag = sensors[sensorID].xmag.filter(function(data) {
			return (curTime - (data.x).getTime()) < historyTime;
		});
	sensors[sensorID].ymag = sensors[sensorID].ymag.filter(function(data) {
			return (curTime - (data.x).getTime()) < historyTime;
		});
	sensors[sensorID].zmag = sensors[sensorID].zmag.filter(function(data) {
			return (curTime - (data.x).getTime()) < historyTime;
		});
	sensors[sensorID].xaccel = sensors[sensorID].xaccel.filter(function(data) {
			return (curTime - (data.x).getTime()) < historyTime;
		});
	sensors[sensorID].yaccel = sensors[sensorID].yaccel.filter(function(data) {
			return (curTime - (data.x).getTime()) < historyTime;
		});
	sensors[sensorID].zaccel = sensors[sensorID].zaccel.filter(function(data) {
			return (curTime - (data.x).getTime()) < historyTime;
		});
}

// parses json blob into javascript object
var addData = function(message, historyTime) {
	// if we've never seen this sensor before	
	if (typeof sensors[message.id] === 'undefined') {
		// create new array to store data from this sensor
		sensors[message.id] = new sensorData(message.id);
	}

	// add new sensorData object with values to sensor id's array of data
	/*sensors[message.id].push(new sensorData(message.id, message.temp*9/5+32, message.rh, message.press, message.pir, message.xmag, message.ymag, message.zmag, message.xaccel, message.yaccel, message.zaccel, message._gatd.time_utc_timestamp));*/
	var dateTime = new Date(message._gatd.time_utc_timestamp/1000);
	/*sensors[message.id].temp.push({x: dateTime, y: message.temp*9/5+32});
	sensors[message.id].rh.push({x: dateTime, y: message.rh});
	sensors[message.id].press.push({x: dateTime, y: message.press});
	sensors[message.id].light.push({x: dateTime, y: message.light});
	sensors[message.id].pir.push({x: dateTime, y: message.pir});
	sensors[message.id].xmag.push({x: dateTime, y: message.xmag});
	sensors[message.id].ymag.push({x: dateTime, y: message.ymag});
	sensors[message.id].zmag.push({x: dateTime, y: message.zmag});
	sensors[message.id].xaccel.push({x: dateTime, y: message.xaccel});
	sensors[message.id].yaccel.push({x: dateTime, y: message.yaccel});
	sensors[message.id].zaccel.push({x: dateTime, y: message.zaccel});*/
	temp_chart.datasets[0].addPoint(dateTime, message.temp*9/5+32);
	pir_chart.datasets[0].addPoint(dateTime, message.pir);
	if (message.xaccel < 0) {
		accel_chart.datasets[0].points[0].value = 0;
		accel_chart.datasets[0].points[3].value = -1*message.xaccel;
	} else {
		accel_chart.datasets[0].points[0].value = message.xaccel;
		accel_chart.datasets[0].points[3].value = 0;
	}
	if (message.yaccel < 0) {
		accel_chart.datasets[0].points[1].value = 0;
		accel_chart.datasets[0].points[4].value = -1*message.yaccel;
	} else {
		accel_chart.datasets[0].points[1].value = message.yaccel;
		accel_chart.datasets[0].points[4].value = 0;
	}
	if (message.zaccel < 0) {
		accel_chart.datasets[0].points[2].value = 0;
		accel_chart.datasets[0].points[5].value = -1*message.zaccel;
	} else {
		accel_chart.datasets[0].points[2].value = message.zaccel;
		accel_chart.datasets[0].points[5].value = 0;
	}


	// for the given sensor id, remove old data points
	//updateSensorData(message.id, historyTime);
}

var updateCharts = function() {
	
}
