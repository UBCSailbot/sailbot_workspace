import custom_interfaces.msg as ci

MEAN_SPEED = ci.HelperSpeed(speed=15.0)  # mean boat speed in kmph
START_POINT = ci.HelperLatLon(latitude=48.46, longitude=-125.1)  # Starting location of the mock
START_HEADING = ci.HelperHeading(heading=180.0)  # in degrees, heading of the boat

'''
For further tests:
START_POINT = ci.HelperLatLon(latitude=49.308157, longitude=-123.244801)
Change last line of mock_global_path.csv to: 49.289686,-123.195877
'''
