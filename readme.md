## ESP8266 Tracer Webserver

[![Build Status](https://travis-ci.org/pfeerick/tracerWebserver.svg?branch=master)](https://travis-ci.org/pfeerick/tracerWebserver)

A simple webserver to create a web dashboard for the Tracer series charge controllers.

You can see the JSON data responses if you naviate to any of the following URLs:

```
  /getRatedData
  /getRealtimeData
  /getRealtimeStatus
  /getStatisticalData
  /getDiscrete
  /getCoils
```

There is also a some system information present at `/info`.

## Screenshots

### Dashboard overview page
![screenshot of dashboard](documentation/images/dashboard.png)

### Realtime Data page
![screenshot of realtime_data page](documentation/images/realtime_data.png)

### Rated Data page
![screenshot of rated_data page](documentation/images/rated_data.png)

### Statistical Data page
![screenshot of statistical_data page](documentation/images/statistical_data.png)

### Coil Data page
![screenshot of coil_data page](documentation/images/coil_data.png)

### Discrete Data page
![screenshot of discrete_data page](documentation/images/discrete_data.png)

### System Info page
![screenshot of System Info page](documentation/images/info.png)
