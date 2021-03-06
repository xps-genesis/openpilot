/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *    \file include/acado/objective/lsq_end_term.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#ifndef ACADO_TOOLKIT_LSQ_END_TERM_HPP
#define ACADO_TOOLKIT_LSQ_END_TERM_HPP


#include <acado/objective/objective_element.hpp>


BEGIN_NAMESPACE_ACADO



/**
 *	\brief Stores and evaluates LSQ mayer terms within optimal control problems.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class LSQEndTerm allows to manage and evaluate least square objective mayer terms  \n
 *  of the general form:                                                            \n
 *                                                                                  \n
 *                0.5* || S * ( m(T,x(T),p(T),...) - r ) ||^2_2                     \n
 *                                                                                  \n
 *  Here the T is the time at the last grid point of the objective grid.            \n
 *
 *  \author Boris Houska, Hans Joachim Ferreau
 */


class LSQEndTerm : public ObjectiveElement{

    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

        /** Default constructor. */
        LSQEndTerm( );

        /** Default constructor. */
        LSQEndTerm( const Grid&     grid_,   /**< the objective grid    */
                    const DMatrix      &S_,   /**< the weighting matrix  */
                    const Function    &m_,   /**< the LSQ function      */
                    const DVector      &r_    /**< the reference vector  */ );

        /** Copy constructor (deep copy). */
        LSQEndTerm( const LSQEndTerm& rhs );

        /** Destructor. */
        virtual ~LSQEndTerm( );

        /** Assignment operator (deep copy). */
        LSQEndTerm& operator=( const LSQEndTerm& rhs );



// =======================================================================================
//
//                                 INITIALIZATION ROUTINES
//
// =======================================================================================


        inline returnValue init( const Grid&     grid_,   /**< the objective grid    */
                                 const DMatrix      &S_,   /**< the weighting matrix  */
                                 const Function&    m_,   /**< the LSQ function      */
                                 const DVector      &r_    /**< the reference vectors */ );




// =======================================================================================
//
//                                   EVALUATION ROUTINES
//
// =======================================================================================


        returnValue evaluate( const OCPiterate &x );



        /** Evaluates the objective gradient contribution from this term \n
         *  and computes the corresponding exact hessian.                \n
         *                                                               \n
         *  \return SUCCESSFUL_RETURN                                    \n
         */
        returnValue evaluateSensitivities( BlockMatrix *hessian );


        /** Evaluates the objective gradient contribution from this term \n
         *  and computes the corresponding GN hessian approximation for  \n
         *  the case that  GNhessian != 0.                               \n
         *                                                               \n
         *  \return SUCCESSFUL_RETURN                                    \n
         */
        returnValue evaluateSensitivitiesGN( BlockMatrix *GNhessian );


        /** returns whether the constraint element is affine. */
        inline BooleanType isAffine();


        /** returns whether the constraint element is convex. */
        inline BooleanType isQuadratic();


        /** returns whether the constraint element is convex. */
        inline BooleanType isConvex();


        /** overwrites the reference vector r    \n
         *                                       \n
         *  \return SUCCESSFUL_RETURN
         */
        inline returnValue setReference( const DVector &ref );


// =======================================================================================


    //
    // DATA MEMBERS:
    //
    protected:

    DMatrix           S              ;    /**< a symmetric weighting matrix */
    DVector           r              ;    /**< a tracking reference         */

    double          *S_h_res        ;    /**< specific intermediate results \n
                                           *  to be stored for backward     \n
                                           *  differentiation               \n
                                           */
};


CLOSE_NAMESPACE_ACADO



#include <acado/objective/lsq_end_term.ipp>

#endif  // ACADO_TOOLKIT_LSQ_END_TERM_HPP

/*
 *     end of file
 */
