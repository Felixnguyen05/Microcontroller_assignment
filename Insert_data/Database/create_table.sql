 USE master;
 GO
IF NOT EXISTS (SELECT name FROM sys.databases WHERE name =
 'final_project_microcontroller')
 BEGIN
 CREATE DATABASE final_project_microcontroller;
 END
 GO
 USE final_project_microcontroller;
 GO
 CREATE TABLE temperature (
 reading_time DATETIME PRIMARY KEY,
 temperature_value FLOAT NOT NULL
 );
 GO

 use final_project_microcontroller
 Go
 Select * from temperature
 truncate table temperature