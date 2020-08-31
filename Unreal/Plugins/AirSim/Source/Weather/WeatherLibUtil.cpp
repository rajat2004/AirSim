#include "WeatherLibUtil.h"
#include "Materials/MaterialParameterCollection.h"
// #include "Runtime/Engine/Classes/Kismet/GameplayStatics.h"
// #include "Blueprint/UserWidget.h"
// #include "Blueprint/WidgetBlueprintLibrary.h"

UWeatherLibUtil::UWeatherLibUtil()
{
    static ConstructorHelpers::FObjectFinder<UMaterialParameterCollection> WeatherParameterCollection(getWeatherParamsObjectPath());

    if (WeatherParameterCollection.Succeeded()) {
        UE_LOG(LogTemp, Warning, TEXT("Succeeded, WeatherAPI could get WeatherParameterCollection!"));
        weather_parameter_collection_ = WeatherParameterCollection.Object;

        // Can be removed
        if (weather_parameter_collection_) {
            UE_LOG(LogTemp, Warning, TEXT("WeatherParameterCollection is not null"));
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("WARNING, WeatherParameterCollection is NULL"));
        }
    }
}
    
UMaterialParameterCollectionInstance* UWeatherLibUtil::getWeatherMaterialCollectionInstance(UWorld* World)
{
    if (World) {
        // UMaterialParameterCollection* WeatherParameterCollection = Cast<UMaterialParameterCollection>(
            // StaticLoadObject(UMaterialParameterCollection::StaticClass(), NULL, getWeatherParamsObjectPath()));

        static ConstructorHelpers::FObjectFinder<UMaterialParameterCollection> WeatherParameterCollection(getWeatherParamsObjectPath());

        // if (WeatherParameterCollection.Succeeded())
        if (weather_parameter_collection_) {
            UMaterialParameterCollectionInstance* Instance = World->GetParameterCollectionInstance(weather_parameter_collection_);
            if (Instance) {
                return Instance;
            }
            else {
                UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get WeatherParameterCollectionInstance!"));
            }
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get WeatherParameterCollection!"));
        }
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get World!"));
    }

    return NULL;
}
