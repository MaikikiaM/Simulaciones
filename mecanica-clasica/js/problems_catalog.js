/**
 * problems_catalog.js — Catálogo unificado de los 13 problemas (6 Sistemas de Partículas + 7 Sólido Rígido).
 */
(function (root, factory) {
    if (typeof module === 'object' && module.exports) {
        module.exports = factory(require('./problems_particles.js'), require('./problems_rigid.js'));
    } else {
        root.ProblemsCatalog = factory(root.ParticleProblems, root.RigidProblems);
    }
}(typeof self !== 'undefined' ? self : this, function (ParticleProblems, RigidProblems) {
    'use strict';

    var all = ParticleProblems.concat(RigidProblems);

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
        particles: ParticleProblems,
        rigid: RigidProblems,
        getDefaultParams: getDefaultParams,
        getById: getById,
        isDefaultPreset: isDefaultPreset
    };
}));
