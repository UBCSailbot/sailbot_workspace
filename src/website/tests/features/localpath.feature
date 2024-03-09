@localpath
Feature: Testing the LocalPath API

    Scenario: Fetch LocalPath data from the API
        Given I clear the database
        And I insert LocalPath data into the database
        When I get all LocalPath interface data
        Then the service success response is 200
        And the response data matches the LocalPath data in the database
