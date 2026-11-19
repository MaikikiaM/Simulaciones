/**
 * problems_catalog.js — Catálogo unificado de los 35 problemas en 4 temas:
 *  - Fundamentos (T1–T4): 4 simulaciones
 *  - Sistemas de Partículas (T5): 13 simulaciones
 *  - Sólido Rígido (T6a): 14 simulaciones
 *  - Elasticidad y Torsión (T6b): 4 simulaciones
 */
(function (root, factory) {
    if (typeof module === 'object' && module.exports) {
        module.exports = factory(
            require('./problems_fundamentals.js'),
            require('./problems_particles.js'),
            require('./problems_rigid.js'),
            require('./problems_elasticity.js')
        );
    } else {
        root.ProblemsCatalog = factory(
            root.FundamentalProblems,
            root.ParticleProblems,
            root.RigidProblems,
            root.ElasticityProblems
        );
    }
}(typeof self !== 'undefined' ? self : this, function (FundamentalProblems, ParticleProblems, RigidProblems, ElasticityProblems) {
    'use strict';

    function assignLevels(arr) {
        for (var i = 0; i < arr.length; i++) {
            arr[i].level = i + 1;
        }
        return arr;
    }

    var fundamentals = assignLevels(FundamentalProblems);
    var particles = assignLevels(ParticleProblems);
    var rigid = assignLevels(RigidProblems);
    var elasticity = assignLevels(ElasticityProblems);
    var all = fundamentals.concat(particles, rigid, elasticity);

    function getDefaultParams(prob) {
        var out = {};
        for (var i = 0; i < prob.paramsSchema.length; i++) {
            var item = prob.paramsSchema[i];
            out[item.key] = item.def;
        }
        return out;
    }

    function getById(id) {
        for (var i = 0; i < all.length; i++) {
            if (all[i].id === id) return all[i];
        }
        return all[0];
    }

    function isDefaultPreset(prob, params) {
        for (var i = 0; i < prob.paramsSchema.length; i++) {
            var item = prob.paramsSchema[i];
            if (Math.abs((params[item.key] || 0) - item.def) > 1e-9) return false;
        }
        return true;
    }

    return {
        all: all,
        fundamentals: fundamentals,
        particles: particles,
        rigid: rigid,
        elasticity: elasticity,
        getDefaultParams: getDefaultParams,
        getById: getById,
        isDefaultPreset: isDefaultPreset
    };
}));
