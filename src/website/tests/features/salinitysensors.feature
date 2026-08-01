@salinity-sensors
Feature: Testing the SalinitySensors API

    Scenario: Fetch SalinitySensors data from the API
        Given I clear the database
        And I insert SalinitySensors data into the database
        When I get all SalinitySensors interface data
        Then the service success response is 200
        And the response data matches the SalinitySensors data in the database
