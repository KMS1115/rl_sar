/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef INFERENCE_RUNTIME_HPP
#define INFERENCE_RUNTIME_HPP

#include <memory>
#include <string>
#include <vector>

#ifdef USE_ONNX
#include <onnxruntime_cxx_api.h>
#endif

namespace InferenceRuntime
{

class Model
{
public:
    virtual ~Model() = default;

    virtual bool load(const std::string& model_path) = 0;
    virtual bool is_loaded() const = 0;
    virtual std::vector<float> forward(const std::vector<std::vector<float>>& inputs) = 0;
    virtual std::string get_model_type() const = 0;
    virtual size_t get_input_count() const = 0;
};

class ONNXModel : public Model
{
private:
    bool loaded_ = false;
    std::string model_path_;

#ifdef USE_ONNX
    std::unique_ptr<Ort::Session> session_;
    std::unique_ptr<Ort::Env> env_;
    Ort::MemoryInfo memory_info_;
    std::vector<std::string> input_node_names_;
    std::vector<std::string> output_node_names_;
    std::vector<std::vector<int64_t>> input_shapes_;
    std::vector<std::vector<int64_t>> output_shapes_;
#endif

public:
    ONNXModel();
    ~ONNXModel();

    bool load(const std::string& model_path) override;
    bool is_loaded() const override { return loaded_; }
    std::vector<float> forward(const std::vector<std::vector<float>>& inputs) override;
    std::string get_model_type() const override { return "onnx"; }
    size_t get_input_count() const override
    {
#ifdef USE_ONNX
        return input_node_names_.size();
#else
        return 0;
#endif
    }

private:
#ifdef USE_ONNX
    void setup_input_output_info();
    std::vector<float> extract_output_data(const std::vector<Ort::Value>& outputs);
#endif
};

class ModelFactory
{
public:
    enum class ModelType
    {
        ONNX,
        AUTO
    };

    static std::unique_ptr<Model> create_model(ModelType type = ModelType::AUTO);
    static ModelType detect_model_type(const std::string& model_path);
    static std::unique_ptr<Model> load_model(const std::string& model_path, ModelType type = ModelType::AUTO);
};

} // namespace InferenceRuntime

#endif // INFERENCE_RUNTIME_HPP
