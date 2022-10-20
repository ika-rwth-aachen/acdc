// Copyright (c) 2022 Institute for Automotive Engineering of RWTH Aachen University

// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#pragma once

//#ifdef CPPADCG

namespace ct {
namespace optcon {

/**
 * \ingroup CostFunction
 *
 * \brief Cost Function with Auto-Diff support and passive parameters
 *
 * Custom AD cost function supporting passive parameters.
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR = double>
class CostFunctionADParams : public CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef core::DerivativesCppadJIT<STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1> JacCG;
    typedef typename JacCG::CG_SCALAR CGScalar;
    typedef Eigen::Matrix<CGScalar, 1, 1> MatrixCg;

    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_state_matrix_t;

    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef Eigen::Matrix<SCALAR, PARAMS_DIM, 1> params_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

    /**
     * \brief Basic constructor
     */
    CostFunctionADParams();

    /**
     * Deep-cloning of cost function
     * @return base pointer to clone
     */
    CostFunctionADParams* clone() const override;

    /**
     * \brief Copy constructor
     * @param arg cost function to copy
     */
    CostFunctionADParams(const CostFunctionADParams& arg);


    /**
     * \brief Destructor
     */
    virtual ~CostFunctionADParams();

    /**
     * @brief      Initializes the AD costfunction, generates and compiles
     *             source code
     */
    virtual void initialize() override;

    /**
     * \brief Add an intermediate, auto-differentiable term
     *
     * Use this function to add an auto-differentiable, intermediate term to the cost function.
     *
     * @param term The term to be added
     * @param verbose Flag enabling printouts
     * @return
     */
    void addIntermediateADTerm(std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
        bool verbose = false) override;

    void addIntermediateADTermParam(std::shared_ptr<TermBase<STATE_DIM+PARAMS_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
        bool verbose = false);

    /**
     * \brief Add a final, auto-differentiable term
     *
     * Use this function to add an auto-differentiable, final term to the cost function.
     *
     * @param term The term to be added
     * @param verbose Flag enabling printouts
     * @return
     */
    void addFinalADTerm(std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
        bool verbose = false) override;

    void addFinalADTermParam(std::shared_ptr<TermBase<STATE_DIM+PARAMS_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
        bool verbose = false);

    void setCurrentStateAndControl(const state_vector_t& x, const control_vector_t& u, const SCALAR& t = 0.0) override;

    void setCurrentParameters(const params_vector_t& p);

    SCALAR evaluateIntermediate() override;
    SCALAR evaluateTerminal() override;

    state_vector_t stateDerivativeIntermediate() override;
    state_vector_t stateDerivativeTerminal() override;

    control_vector_t controlDerivativeIntermediate() override;
    control_vector_t controlDerivativeTerminal() override;

    state_matrix_t stateSecondDerivativeIntermediate() override;
    state_matrix_t stateSecondDerivativeTerminal() override;

    control_matrix_t controlSecondDerivativeIntermediate() override;
    control_matrix_t controlSecondDerivativeTerminal() override;

    control_state_matrix_t stateControlDerivativeIntermediate() override;
    control_state_matrix_t stateControlDerivativeTerminal() override;


private:
    MatrixCg evaluateIntermediateCg(const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1>& stateInputTime);
    MatrixCg evaluateTerminalCg(const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1>& stateInputTime);

    //! combined state, control, time and parameter vector
    Eigen::Matrix<SCALAR, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1> stateParamsControlTime_;

    //! intermediate AD terms
    std::vector<std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>>>            intermediateTerms_;
    std::vector<std::shared_ptr<TermBase<STATE_DIM+PARAMS_DIM, CONTROL_DIM, SCALAR, CGScalar>>> intermediateTermsParam_;
    //! final AD terms
    std::vector<std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>>>            finalTerms_;
    std::vector<std::shared_ptr<TermBase<STATE_DIM+PARAMS_DIM, CONTROL_DIM, SCALAR, CGScalar>>> finalTermsParam_;

    //! generated jacobians
    std::shared_ptr<JacCG> intermediateCostCodegen_;
    std::shared_ptr<JacCG> finalCostCodegen_;

    //! cppad functions
    typename JacCG::FUN_TYPE_CG intermediateFun_;
    typename JacCG::FUN_TYPE_CG finalFun_;
};

}  // namespace optcon
}  // namespace ct

//#endif
