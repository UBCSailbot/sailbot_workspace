@temp-sensors
Feature: Testing the TempSensors API

    Scenario: Fetch TempSensors data from the API
        Given I clear the database
        And I insert TempSensors data into the database
        When I get all TempSensors interface data
        Then the service success response is 200
        And the response data matches the TempSensors data in the database
