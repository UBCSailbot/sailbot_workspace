@generic-sensors
Feature: Testing the GenericSensors API

    Scenario: Fetch GenericSensors data from the API
        Given I clear the database
        And I insert GenericSensors data into the database
        When I get all GenericSensors interface data
        Then the service success response is 200
        And the response data matches the GenericSensors data in the database
