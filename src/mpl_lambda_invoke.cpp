#include <aws/lambda-runtime/runtime.h>
#include <aws/core/Aws.h>
#include <aws/core/utils/Outcome.h>
#include <aws/lambda/LambdaClient.h>
#include <aws/lambda/model/InvokeRequest.h>
#include <aws/core/utils/json/JsonSerializer.h>
#include <fstream>
#include <iostream>

static const char* ALLOCATION_TAG = "mplLambdaAWS";

static std::shared_ptr<Aws::Lambda::LambdaClient> m_client;

void invokeLambda() {
    Aws::Lambda::Model::InvokeRequest invokeRequest;
    invokeRequest.SetFunctionName("mpl_fixed_graph_aws");
    invokeRequest.SetInvocationType(Aws::Lambda::Model::InvocationType::Event);
    std::shared_ptr<Aws::IOStream> payload = Aws::MakeShared<Aws::StringStream>("PayloadData");
    Aws::Utils::Json::JsonValue jsonPayload;
// --scenario png --start 500,10 --goal 500,20 --global_min 0,0 --global_max 2000,1300 --num_samples 1000 --env ./resources/house_layout.png --lambda_id 0 --num_divisions 1,1
    jsonPayload.WithString("scenario", "png");
    jsonPayload.WithString("coordinator", "54.188.233.199");
    jsonPayload.WithString("start", "500,10;1000,500");
    jsonPayload.WithString("goal", "500,500;100,1300");
    jsonPayload.WithString("global_min", "0,0");
    jsonPayload.WithString("global_max", "1403,1404");
    jsonPayload.WithString("algorithm", "prm_fixed_graph");
    jsonPayload.WithString("env", "resource/house_layout.png");
    jsonPayload.WithString("lambda_id", "0");
    jsonPayload.WithString("num_divisions", "1,1");
    jsonPayload.WithString("robot", "");
    jsonPayload.WithString("envFrame", "");
    *payload << jsonPayload.View().WriteReadable();
    invokeRequest.SetBody(payload);
    invokeRequest.SetContentType("application/json");

    auto outcome = m_client->Invoke(invokeRequest);
    if (outcome.IsSuccess())
    {
        auto &result = outcome.GetResult();
        Aws::IOStream& payload = result.GetPayload();
        Aws::String functionResult;
        std::getline(payload, functionResult);
        std::cout << "Lambda result:\n" << functionResult << "\n\n";
    }
    else {
        auto &error = outcome.GetError();
        std::cout << "Error: " << error.GetExceptionName() << "\nMessage: " << error.GetMessage() << "\n\n";
    }
}

int main(int argc, char *argv[]) {
    Aws::SDKOptions options;
    Aws::InitAPI(options);
    Aws::Client::ClientConfiguration clientConfig;
    clientConfig.region = "us-west-2";
    m_client = Aws::MakeShared<Aws::Lambda::LambdaClient>(ALLOCATION_TAG, clientConfig);
    invokeLambda();
    Aws::ShutdownAPI(options);
    return 0;
}

