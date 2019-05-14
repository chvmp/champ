#ifndef _ROS_sensor_msgs_BatteryState_h
#define _ROS_sensor_msgs_BatteryState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace sensor_msgs
{

  class BatteryState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _voltage_type;
      _voltage_type voltage;
      typedef float _current_type;
      _current_type current;
      typedef float _charge_type;
      _charge_type charge;
      typedef float _capacity_type;
      _capacity_type capacity;
      typedef float _design_capacity_type;
      _design_capacity_type design_capacity;
      typedef float _percentage_type;
      _percentage_type percentage;
      typedef uint8_t _power_supply_status_type;
      _power_supply_status_type power_supply_status;
      typedef uint8_t _power_supply_health_type;
      _power_supply_health_type power_supply_health;
      typedef uint8_t _power_supply_technology_type;
      _power_supply_technology_type power_supply_technology;
      typedef bool _present_type;
      _present_type present;
      uint32_t cell_voltage_length;
      typedef float _cell_voltage_type;
      _cell_voltage_type st_cell_voltage;
      _cell_voltage_type * cell_voltage;
      typedef const char* _location_type;
      _location_type location;
      typedef const char* _serial_number_type;
      _serial_number_type serial_number;
      enum { POWER_SUPPLY_STATUS_UNKNOWN =  0 };
      enum { POWER_SUPPLY_STATUS_CHARGING =  1 };
      enum { POWER_SUPPLY_STATUS_DISCHARGING =  2 };
      enum { POWER_SUPPLY_STATUS_NOT_CHARGING =  3 };
      enum { POWER_SUPPLY_STATUS_FULL =  4 };
      enum { POWER_SUPPLY_HEALTH_UNKNOWN =  0 };
      enum { POWER_SUPPLY_HEALTH_GOOD =  1 };
      enum { POWER_SUPPLY_HEALTH_OVERHEAT =  2 };
      enum { POWER_SUPPLY_HEALTH_DEAD =  3 };
      enum { POWER_SUPPLY_HEALTH_OVERVOLTAGE =  4 };
      enum { POWER_SUPPLY_HEALTH_UNSPEC_FAILURE =  5 };
      enum { POWER_SUPPLY_HEALTH_COLD =  6 };
      enum { POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE =  7 };
      enum { POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE =  8 };
      enum { POWER_SUPPLY_TECHNOLOGY_UNKNOWN =  0 };
      enum { POWER_SUPPLY_TECHNOLOGY_NIMH =  1 };
      enum { POWER_SUPPLY_TECHNOLOGY_LION =  2 };
      enum { POWER_SUPPLY_TECHNOLOGY_LIPO =  3 };
      enum { POWER_SUPPLY_TECHNOLOGY_LIFE =  4 };
      enum { POWER_SUPPLY_TECHNOLOGY_NICD =  5 };
      enum { POWER_SUPPLY_TECHNOLOGY_LIMN =  6 };

    BatteryState():
      header(),
      voltage(0),
      current(0),
      charge(0),
      capacity(0),
      design_capacity(0),
      percentage(0),
      power_supply_status(0),
      power_supply_health(0),
      power_supply_technology(0),
      present(0),
      cell_voltage_length(0), cell_voltage(NULL),
      location(""),
      serial_number("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.real = this->voltage;
      *(outbuffer + offset + 0) = (u_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.real = this->current;
      *(outbuffer + offset + 0) = (u_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_charge;
      u_charge.real = this->charge;
      *(outbuffer + offset + 0) = (u_charge.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_charge.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_charge.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_charge.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->charge);
      union {
        float real;
        uint32_t base;
      } u_capacity;
      u_capacity.real = this->capacity;
      *(outbuffer + offset + 0) = (u_capacity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_capacity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_capacity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_capacity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->capacity);
      union {
        float real;
        uint32_t base;
      } u_design_capacity;
      u_design_capacity.real = this->design_capacity;
      *(outbuffer + offset + 0) = (u_design_capacity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_design_capacity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_design_capacity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_design_capacity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->design_capacity);
      union {
        float real;
        uint32_t base;
      } u_percentage;
      u_percentage.real = this->percentage;
      *(outbuffer + offset + 0) = (u_percentage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_percentage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_percentage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_percentage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->percentage);
      *(outbuffer + offset + 0) = (this->power_supply_status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->power_supply_status);
      *(outbuffer + offset + 0) = (this->power_supply_health >> (8 * 0)) & 0xFF;
      offset += sizeof(this->power_supply_health);
      *(outbuffer + offset + 0) = (this->power_supply_technology >> (8 * 0)) & 0xFF;
      offset += sizeof(this->power_supply_technology);
      union {
        bool real;
        uint8_t base;
      } u_present;
      u_present.real = this->present;
      *(outbuffer + offset + 0) = (u_present.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->present);
      *(outbuffer + offset + 0) = (this->cell_voltage_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cell_voltage_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cell_voltage_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cell_voltage_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell_voltage_length);
      for( uint32_t i = 0; i < cell_voltage_length; i++){
      union {
        float real;
        uint32_t base;
      } u_cell_voltagei;
      u_cell_voltagei.real = this->cell_voltage[i];
      *(outbuffer + offset + 0) = (u_cell_voltagei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell_voltagei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cell_voltagei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cell_voltagei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell_voltage[i]);
      }
      uint32_t length_location = strlen(this->location);
      varToArr(outbuffer + offset, length_location);
      offset += 4;
      memcpy(outbuffer + offset, this->location, length_location);
      offset += length_location;
      uint32_t length_serial_number = strlen(this->serial_number);
      varToArr(outbuffer + offset, length_serial_number);
      offset += 4;
      memcpy(outbuffer + offset, this->serial_number, length_serial_number);
      offset += length_serial_number;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.base = 0;
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage = u_voltage.real;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.base = 0;
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current = u_current.real;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_charge;
      u_charge.base = 0;
      u_charge.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_charge.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_charge.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_charge.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->charge = u_charge.real;
      offset += sizeof(this->charge);
      union {
        float real;
        uint32_t base;
      } u_capacity;
      u_capacity.base = 0;
      u_capacity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_capacity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_capacity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_capacity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->capacity = u_capacity.real;
      offset += sizeof(this->capacity);
      union {
        float real;
        uint32_t base;
      } u_design_capacity;
      u_design_capacity.base = 0;
      u_design_capacity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_design_capacity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_design_capacity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_design_capacity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->design_capacity = u_design_capacity.real;
      offset += sizeof(this->design_capacity);
      union {
        float real;
        uint32_t base;
      } u_percentage;
      u_percentage.base = 0;
      u_percentage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_percentage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_percentage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_percentage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->percentage = u_percentage.real;
      offset += sizeof(this->percentage);
      this->power_supply_status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->power_supply_status);
      this->power_supply_health =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->power_supply_health);
      this->power_supply_technology =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->power_supply_technology);
      union {
        bool real;
        uint8_t base;
      } u_present;
      u_present.base = 0;
      u_present.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->present = u_present.real;
      offset += sizeof(this->present);
      uint32_t cell_voltage_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cell_voltage_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cell_voltage_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cell_voltage_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cell_voltage_length);
      if(cell_voltage_lengthT > cell_voltage_length)
        this->cell_voltage = (float*)realloc(this->cell_voltage, cell_voltage_lengthT * sizeof(float));
      cell_voltage_length = cell_voltage_lengthT;
      for( uint32_t i = 0; i < cell_voltage_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_cell_voltage;
      u_st_cell_voltage.base = 0;
      u_st_cell_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cell_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cell_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cell_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_cell_voltage = u_st_cell_voltage.real;
      offset += sizeof(this->st_cell_voltage);
        memcpy( &(this->cell_voltage[i]), &(this->st_cell_voltage), sizeof(float));
      }
      uint32_t length_location;
      arrToVar(length_location, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_location; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_location-1]=0;
      this->location = (char *)(inbuffer + offset-1);
      offset += length_location;
      uint32_t length_serial_number;
      arrToVar(length_serial_number, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_serial_number; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_serial_number-1]=0;
      this->serial_number = (char *)(inbuffer + offset-1);
      offset += length_serial_number;
     return offset;
    }

    const char * getType(){ return "sensor_msgs/BatteryState"; };
    const char * getMD5(){ return "476f837fa6771f6e16e3bf4ef96f8770"; };

  };

}
#endif