// AirSim Weather API Utility library

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Materials/MaterialParameterCollectionInstance.h"

class UWeatherLibUtil
{
public:
    UWeatherLibUtil();
    UMaterialParameterCollectionInstance* getWeatherMaterialCollectionInstance(UWorld* World);

private:
    // not sure why, but content folder should be omitted in the path
    // location of the weather UMaterialParameterCollection, params for rain snow wind etc
    static const TCHAR* getWeatherParamsObjectPath()
    {
        return TEXT("/AirSim/Weather/WeatherFX/WeatherGlobalParams");
    }

private:
    UMaterialParameterCollection* weather_parameter_collection_;
};
